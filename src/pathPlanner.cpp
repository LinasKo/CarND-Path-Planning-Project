#include "pathPlanner.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include "spdlog/spdlog.h"
#include "spline/spline.h"

#include "helpers.h"


using namespace path_planning;

static constexpr double LANE_CHANGE_COST = 1.0;  // Penalty to speed when deciding whether to change to a faster lane

static constexpr double PATH_DURATION_SECONDS = 2.5;  // How long should the traversal of the generated path take
static constexpr double NODE_TRAVERSAL_RATE_SECONDS = 0.02;  // How much time passes in the simulation between each node of the given path

static constexpr double MPH_TO_METRES_PER_SECOND(const double mph)  // Conversion from MPH to units in the simulator (metres per second)
{
    return mph / 3600.0 * 1609.344;
}

static constexpr double SPEED_LIMIT_METRES_PER_SECOND = MPH_TO_METRES_PER_SECOND(50.0 * 0.9);  // Speed limit

static constexpr double MAX_ACCELERATION = 10.0 * 0.5;  // Maximum allowed acceleration of the vehicle.
// The previous values are difficult to hard-cap when min-jerk trajectories are generated and total duration of path traversal is provided.
// Therefore, reducing by a ratio, determined by trial-and-error.
static constexpr double MAX_VELOCITY_CHANGE = 3.0;  // Dampen velocity change passed to the polynomial trajectory generator if it's greater than this.

static constexpr unsigned TRAJECTORY_HISTORY_LENGTH = 100u; // How many nodes from a trajectory to keep in history

static constexpr double LANE_SPEED_FORWARD_SCAN_RANGE = SPEED_LIMIT_METRES_PER_SECOND * 3.0;  // How far ahead to look for another vehicle when determining lane speeds
// static constexpr double LANE_SPEED_BACKWARD_SCAN_RANGE = SPEED_LIMIT_METRES_PER_SECOND * 0.25;  // How far behind to look for another vehicle when determining lane speeds
static constexpr double LANE_CHANGE_CLEAR = SPEED_LIMIT_METRES_PER_SECOND * 0.5;  // How far behind and ahead to look into other lanes to see if another vehicle is blocking a lane change

static constexpr unsigned int LANE_CHANGE_PENALTY = 5u;  // How many ticks to prevent the vehicle from changing lanes after doing so, or when distance to desired lane is too great
static constexpr double D_LIMIT_FOR_LANE_CHANGE_PENALTY = 0.5;  // What's the maximum allowed d distance to a lane before a lange change penalty is imposed


PathPlanner::PathPlanner(std::vector<Waypoint> waypoints) :
    m_waypoints(waypoints)
{}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::planPath(const SimulatorResponseData& simulatorData)
{
    updateHistory(simulatorData);
    spdlog::trace("[planPath] EgoCar: x={}, y={}, s={}, d={}", simulatorData.egoCar.x, simulatorData.egoCar.y, simulatorData.egoCar.s, simulatorData.egoCar.d);

    const std::array<double, 3> laneSpeeds = getLaneSpeeds(simulatorData.egoCar, simulatorData.otherCars);
    spdlog::trace("[planPath] Lane Speeds: {}, {}, {}", laneSpeeds[0], laneSpeeds[1], laneSpeeds[2]);

    // Keep a counter that attempts to track when a car has completed a lane change manoeuvre
    // Side effect of low penalty tolerance: no overtaking on the curved road
    if (std::abs(simulatorData.egoCar.d - m_targetLaneD) > D_LIMIT_FOR_LANE_CHANGE_PENALTY)
    {
        m_laneChangeDelay = LANE_CHANGE_PENALTY;
        spdlog::trace("[planPath] Applied lane change penalty for d distance of {}.", std::abs(simulatorData.egoCar.d - m_targetLaneD));
    }
    else if (m_laneChangeDelay != 0)
    {
        --m_laneChangeDelay;
        spdlog::trace("[planPath] Remaining lane change penalty: {}", m_laneChangeDelay);
    }
    else
    {
        // Allow lane change
        if (m_targetLaneD == D_LEFT_LANE && laneSpeeds[1] - LANE_CHANGE_COST > laneSpeeds[0] && not isLaneBlocked(1, simulatorData.egoCar, simulatorData.otherCars))
        {
            spdlog::info("[planPath] Changing to middle lane.");
            m_targetLaneD = D_MIDDLE_LANE;
            m_targetLaneIndex = 1;
            m_laneChangeDelay = LANE_CHANGE_PENALTY;
        }
        else if (m_targetLaneD == D_RIGHT_LANE && laneSpeeds[1] - LANE_CHANGE_COST > laneSpeeds[2] && not isLaneBlocked(1, simulatorData.egoCar, simulatorData.otherCars))
        {
            spdlog::info("[planPath] Changing to middle lane.");
            m_targetLaneD = D_MIDDLE_LANE;
            m_targetLaneIndex = 1;
            m_laneChangeDelay = LANE_CHANGE_PENALTY;
        }
        else if (m_targetLaneD == D_MIDDLE_LANE && (laneSpeeds[0] - LANE_CHANGE_COST > laneSpeeds[1] || laneSpeeds[2] - LANE_CHANGE_COST > laneSpeeds[1]))
        {
            if (laneSpeeds[0] > laneSpeeds[2] && not isLaneBlocked(0, simulatorData.egoCar, simulatorData.otherCars))
            {
                spdlog::info("[planPath] Changing to left lane.");
                m_targetLaneD = D_LEFT_LANE;
                m_targetLaneIndex = 0;
                m_laneChangeDelay = LANE_CHANGE_PENALTY;
            }
            else if (not isLaneBlocked(2, simulatorData.egoCar, simulatorData.otherCars))
            {
                spdlog::info("[planPath] Changing to right lane.");
                m_targetLaneD = D_RIGHT_LANE;
                m_targetLaneIndex = 2;
                m_laneChangeDelay = LANE_CHANGE_PENALTY;
            }
        }
    }

    const unsigned keepPrev = 10;
    std::pair<std::vector<double>, std::vector<double>> xyTrajectory = genPathSpline(
        simulatorData.egoCar, laneSpeeds[m_targetLaneIndex], simulatorData.prevPathX, simulatorData.prevPathY, keepPrev);

    m_prevSentX = xyTrajectory.first;
    m_prevSentY = xyTrajectory.second;

    spdlog::debug("[planPath] ------------------------------");

    return xyTrajectory;
}

void PathPlanner::updateHistory(const SimulatorResponseData& simulatorData)
{
    assert(m_prevSentX.size() == m_prevSentY.size());
    assert(simulatorData.prevPathX.size() == simulatorData.prevPathY.size());

    const int executedCommandsCount = m_prevSentX.size() - simulatorData.prevPathX.size();

    for (auto it = m_prevSentX.begin(); it != m_prevSentX.begin() + executedCommandsCount; ++it)
    {
        m_historyEgoX.push_front(*it);
    }
    if (m_historyEgoX.size() > TRAJECTORY_HISTORY_LENGTH)
    {
        m_historyEgoX.resize(TRAJECTORY_HISTORY_LENGTH);
    }

    for (auto it = m_prevSentY.begin(); it != m_prevSentY.begin() + executedCommandsCount; ++it)
    {
        m_historyEgoY.push_front(*it);
    }
    if (m_historyEgoY.size() > TRAJECTORY_HISTORY_LENGTH)
    {
        m_historyEgoY.resize(TRAJECTORY_HISTORY_LENGTH);
    }

    spdlog::trace("[updateHistory] XY history queue size is now ({}, {})", m_historyEgoX.size(), m_historyEgoY.size());
}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::genPathSpline(
    const EgoCar& egoCar, const double maxLaneSpeed, const std::vector<double>& prevPathX, const std::vector<double>& prevPathY, const unsigned keepPrevious)
{
    // Because of the issues in getFrenet function, I will not fully trust the frenet coordinates and will try to avoid them if I can.
    // Still, we need them to compute the destination, so they will be used in 2 cases:
    // 1. The initial s position of the vehicle. This does not use getFrenet, but obtains it from the simulator, with hopefully better accuracy.
    // 2. The goal s position for the vehicle. If the coordinate is incorrect, the goal will be far enough to be corrected at a later timepoint.

    assert(prevPathX.size() == prevPathY.size());
    const unsigned prevAvailable = std::min((unsigned)prevPathX.size(), keepPrevious);

    spdlog::trace("Prev path:");
    spdlog::trace("X:  {}\n", toString(prevPathX));
    spdlog::trace("Y:  {}\n", toString(prevPathY));

    // Set up spline keypoints
    spdlog::debug("[genPathSpline] maxLaneSpeed = {}, startSpeed + MAX_ACCELERATION = {}", maxLaneSpeed, MPH_TO_METRES_PER_SECOND(egoCar.speed) + MAX_ACCELERATION);
    const double maxSpeed = std::min(maxLaneSpeed, MPH_TO_METRES_PER_SECOND(egoCar.speed) + MAX_ACCELERATION);
    // const double maxSpeed = maxLaneSpeed;

    // Just because spline needs > 2 points
    const double dDifference = m_targetLaneD - egoCar.d;
    double beforeEndS = egoCar.s + maxSpeed * PATH_DURATION_SECONDS * 0.5;
    double beforeEndD = egoCar.d + dDifference * 0.5;

    double endS = egoCar.s + maxSpeed * PATH_DURATION_SECONDS;
    double endD = m_targetLaneD;

    double beforeEndX, beforeEndY, endX, endY;
    std::tie(beforeEndX, beforeEndY) = getXY(beforeEndS, beforeEndD, m_waypoints);
    std::tie(endX, endY) = getXY(endS, endD, m_waypoints);

    // Prepend some points of the car position history
    std::vector<double> xPoints, yPoints;

    double xReference = egoCar.x;
    double yReference = egoCar.y;

    auto xHistIter = m_historyEgoX.begin();  // start from most recent
    auto yHistIter = m_historyEgoY.begin();
    for (; xHistIter != m_historyEgoX.end() && xHistIter != m_historyEgoY.end(); ++xHistIter, ++yHistIter)
    {
        double histX = *xHistIter;
        double histY = *yHistIter;
        const double distToRef = distance(histX, histY, xReference, yReference);

        // Don't consider points in history that are too close to one another - it messes up the splines
        spdlog::trace("Considering historical ({}, {}) to prepend to path. Distance to reference = {}.", histX, histY, distToRef);
        if (distToRef > 1.5)
        {
            spdlog::trace("Prepended.");
            xPoints.insert(xPoints.begin(), histX);
            yPoints.insert(yPoints.begin(), histY);

            xReference = histX;
            yReference = histY;
        };
    }

    // Push starting position
    xPoints.push_back(egoCar.x);
    yPoints.push_back(egoCar.y);

    // Add prev points, returned by the simulator
    if (prevPathX.size() >= 5)
    {
        xPoints.push_back(prevPathX[4]);
        yPoints.push_back(prevPathY[4]);
    }
    if (prevPathX.size() >= 10)
    {
        xPoints.push_back(prevPathX[9]);
        yPoints.push_back(prevPathY[9]);
    }

    // Push endpoints
    xPoints.push_back(beforeEndX);
    xPoints.push_back(endX);
    yPoints.push_back(beforeEndY);
    yPoints.push_back(endY);

    spdlog::trace("xPoints:  {}\n", toString(xPoints));
    spdlog::trace("yPoints:  {}\n", toString(yPoints));

    // Initialise and target to reach if this is the first run or pick a new target if the old one is reached.
    const double distToTarget = distance(egoCar.x, egoCar.y, m_targetX, m_targetY);
    spdlog::debug("[genPathSpline] Ego car (x, y)=({}, {}). distToTarget={}",  egoCar.x, egoCar.y, distToTarget);
    if (m_targetX == std::numeric_limits<double>::min() || m_targetY == std::numeric_limits<double>::min() ||
        distToTarget < 5.0)
    {
        m_targetX = xPoints[xPoints.size() - 1];
        m_targetY = yPoints[yPoints.size() - 1];
        spdlog::info("[genPathSpline] Setting new target: ({}, {})",  m_targetX, m_targetY);
    }

    std::vector<double> xCarPoints, yCarPoints;
    std::tie(xCarPoints, yCarPoints) = worldCoordToCarCoord(egoCar, xPoints, yPoints);

    spdlog::trace("local car xPoints:  {}\n", toString(xCarPoints));
    spdlog::trace("local car yPoints:  {}\n", toString(yCarPoints));

    tk::spline spl;
    if (not std::is_sorted(xCarPoints.begin(), xCarPoints.end()))
    {
        spdlog::error("[genPathSpline] xCarPoints are not sorted:  {}", toString(xCarPoints));
        assert(false);
    }
    spl.set_points(xCarPoints, yCarPoints);

    double xTargetCarPoint, yTargetCarPoint;
    std::tie(xTargetCarPoint, yTargetCarPoint) = worldCoordToCarCoord(egoCar, m_targetX, m_targetY);

    xCarPoints.clear();
    yCarPoints.clear();

    // Make path from spline

    const auto numCommandsExecuted = m_prevSentX.size() - prevPathX.size();

    static constexpr double SPEED_INCREASE = 0.1;  // Manually tuned
    static double prevSpeed { MPH_TO_METRES_PER_SECOND(egoCar.speed) };  // Will be 0.0, but just in case

    const double prevEndSpeed = prevSpeed + SPEED_INCREASE * numCommandsExecuted;
    spdlog::trace("[genPathSpline] car Speed={}, prev speed? = {}", MPH_TO_METRES_PER_SECOND(egoCar.speed), prevEndSpeed);  // TODO: this can be used to plot charts of egoCar.speed and actual speed discrepancies
    // prevSpeed = MPH_TO_METRES_PER_SECOND(egoCar.speed);
    prevSpeed = prevEndSpeed;

    // So, the returned car speed is bull***t. Sigh. I keep getting it as a steadily increasing number and then suddenly it goes down to some low number, before picking up again.
    // Better cache the speed I'm setting. And I hope I don't forget to mention this in the writeup.

    int i = 0;
    for (double t = NODE_TRAVERSAL_RATE_SECONDS; t < PATH_DURATION_SECONDS; t += NODE_TRAVERSAL_RATE_SECONDS, ++i)
    {
        const double speedDiff = maxLaneSpeed - prevEndSpeed;
        const double speedChangeSign = speedDiff >= 0 ? 1.0 : -1.0;

        double newSpeed = prevEndSpeed + speedChangeSign * SPEED_INCREASE * i;
        if (newSpeed > SPEED_LIMIT_METRES_PER_SECOND)
        {
            newSpeed = SPEED_LIMIT_METRES_PER_SECOND;
        }
        else if (newSpeed < 0.0)
        {
            newSpeed = 0.0;
        }

        // double newSpeed = prevEndSpeed + SPEED_INCREASE * i;
        // if (newSpeed > SPEED_LIMIT_METRES_PER_SECOND)
        // {
        //     newSpeed = SPEED_LIMIT_METRES_PER_SECOND;
        // }
        const double x = newSpeed * t;

        xCarPoints.push_back(x);
        yCarPoints.push_back(spl(x));
    }

    spdlog::trace("Car path X:  {}\n", toString(xCarPoints));
    spdlog::trace("Car path path Y:  {}\n", toString(yCarPoints));

    std::tie(xPoints, yPoints) = carCoordToWorldCoord(egoCar, xCarPoints, yCarPoints);

    spdlog::trace("Sending path X:  {}\n", toString(xPoints));
    spdlog::trace("Sending path Y:  {}\n", toString(yPoints));

    return std::make_pair(xPoints, yPoints);
}

int PathPlanner::getCarAhead(const EgoCar& egoCar, const std::vector<OtherCar>& otherCars) const
{
    int carAheadIndex = -1;
    double minSDist = std::numeric_limits<double>::max();

    for (int i = 0; i < otherCars.size(); ++i)
    {
        const OtherCar& otherCar = otherCars[i];

        // TODO: does not account for wrap-around in S
        if (std::abs(otherCar.d - egoCar.d) < 1.0 and otherCar.s >= egoCar.s)
        {
            double sDist = otherCar.s - egoCar.s;
            if (sDist < minSDist)
            {
                minSDist = sDist;
                carAheadIndex = i;
            }
        }
    }

    return carAheadIndex;
}

int PathPlanner::getCarBehind(const EgoCar& egoCar, const std::vector<OtherCar>& otherCars) const
{
    int carBehindIndex = -1;
    double minSDist = std::numeric_limits<double>::max();

    for (int i = 0; i < otherCars.size(); ++i)
    {
        const OtherCar& otherCar = otherCars[i];

        // TODO: does not account for wrap-around in S
        if (std::abs(otherCar.d - egoCar.d) < 1.0 and otherCar.s < egoCar.s)
        {
            double sDist = egoCar.s - otherCar.s;
            if (sDist < minSDist)
            {
                minSDist = sDist;
                carBehindIndex = i;
            }
        }
    }

    return carBehindIndex;
}

bool PathPlanner::isLaneBlocked(const int targetLaneIndex, const EgoCar& egoCar, const std::vector<OtherCar>& otherCars) const
{
    for (const auto& otherCar : otherCars)
    {
        // TODO: address S wraparound
        if (std::abs(otherCar.d - egoCar.d) < 1.0 && std::abs(otherCar.s - egoCar.s) <= LANE_CHANGE_CLEAR)
        {
            return true;
        }
    }
    return false;
}

std::array<double, 3> PathPlanner::getLaneSpeeds(EgoCar egoCar, const std::vector<OtherCar>& otherCars) const
{
    std::array<double, 3> speeds;
    const std::array<double, 3> lanes {D_LEFT_LANE, D_MIDDLE_LANE, D_RIGHT_LANE};

    for (auto i = 0; i < lanes.size(); ++i)
    {
        egoCar.d = lanes[i];
        int carAheadIndex = getCarAhead(egoCar, otherCars);
        int carBehindIndex = getCarBehind(egoCar, otherCars);
        spdlog::trace("[getLaneSpeeds] Car ahead: {}", carAheadIndex);

        double carAheadSpeed = SPEED_LIMIT_METRES_PER_SECOND;
        double carBehindSpeed = SPEED_LIMIT_METRES_PER_SECOND;

        if (carAheadIndex != -1 && otherCars[carAheadIndex].s - egoCar.s <= LANE_SPEED_FORWARD_SCAN_RANGE)
        {
            const auto& carAhead = otherCars[carAheadIndex];
            carAheadSpeed = std::sqrt(carAhead.dx * carAhead.dx + carAhead.dy * carAhead.dy);
        }
        // if (carBehindIndex != -1 && egoCar.s - otherCars[carBehindIndex].s <= LANE_SPEED_BACKWARD_SCAN_RANGE)
        // {
        //     // TODO: simply prevent lane change?
        //     const auto& carBehind = otherCars[carBehindIndex];
        //     carBehindSpeed = std::sqrt(carBehind.dx * carBehind.dx + carBehind.dy * carBehind.dy);
        // }

        speeds[i] = std::min(carAheadSpeed, carBehindSpeed);
    }

    return speeds;
}

std::vector<OtherCar> PathPlanner::predictCars(const std::vector<OtherCar>& otherCars, double deltaTime)
{
    std::vector<OtherCar> predictions;
    predictions.reserve(otherCars.size());

    std::transform(otherCars.begin(), otherCars.end(), predictions.begin(), [this, deltaTime](OtherCar otherCar){
        otherCar.x += deltaTime * otherCar.dx;
        otherCar.y += deltaTime * otherCar.dy;

        double heading = orientation(otherCar.dy, otherCar.dx);
        double newS, newD;
        std::tie(newS, newD) = getFrenet(otherCar.x, otherCar.y, heading, m_waypoints);

        otherCar.s = newS;
        otherCar.d = newD;

        // Predicting that otherCar.dx stays the same
        // Predicting that otherCar.dy stays the same

        return otherCar;
    });

    return predictions;
}

std::pair<double, double> PathPlanner::worldCoordToCarCoord(const EgoCar& egoCar, double worldX, double worldY)
{
    const double carYawRadians = M_PI * egoCar.yaw / 180.0;

    double shiftX = worldX - egoCar.x;
    double shiftY = worldY - egoCar.y;

    double carX = shiftX * std::cos(-carYawRadians) - shiftY * std::sin(-carYawRadians);
    double carY = shiftX * std::sin(-carYawRadians) + shiftY * std::cos(-carYawRadians);

    return std::make_pair(carX, carY);
}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::worldCoordToCarCoord(const EgoCar& egoCar, std::vector<double> xsWorld, std::vector<double> ysWorld)
{
    assert(xsWorld.size() == ysWorld.size());

    std::vector<double> xsCar(xsWorld.size()), ysCar(xsWorld.size());
    for (auto i = 0; i < xsWorld.size(); ++i)
    {
        double xCar, yCar;
        std::tie(xCar, yCar) = worldCoordToCarCoord(egoCar, xsWorld[i], ysWorld[i]);
        xsCar[i] = xCar;
        ysCar[i] = yCar;
    }

    return std::make_pair(xsCar, ysCar);
}

std::pair<double, double> PathPlanner::carCoordToWorldCoord(const EgoCar& egoCar, double carX, double carY)
{
    const double carYawRadians = M_PI * egoCar.yaw / 180.0;

    double worldX = carX * std::cos(carYawRadians) - carY * std::sin(carYawRadians);
    double worldY = carX * std::sin(carYawRadians) + carY * std::cos(carYawRadians);

    worldX += egoCar.x;
    worldY += egoCar.y;

    return std::make_pair(worldX, worldY);
}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::carCoordToWorldCoord(const EgoCar& egoCar, std::vector<double> xsCar, std::vector<double> ysCar)
{
    assert(xsCar.size() == ysCar.size());

    std::vector<double> xsWorld(ysCar.size()), ysWorld(xsCar.size());
    for (auto i = 0; i < xsCar.size(); ++i)
    {
        double xWorld, yWorld;
        std::tie(xWorld, yWorld) = carCoordToWorldCoord(egoCar, xsCar[i], ysCar[i]);
        xsWorld[i] = xWorld;
        ysWorld[i] = yWorld;
    }

    return std::make_pair(xsWorld, ysWorld);
}

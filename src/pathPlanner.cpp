#include "pathPlanner.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <tuple>
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

static constexpr unsigned TRAJECTORY_HISTORY_LENGTH = 5u; // How many nodes from a trajectory to keep in history
static_assert(TRAJECTORY_HISTORY_LENGTH >= 4);  // Need at least this many to compute kinematics

static constexpr double LANE_SPEED_FORWARD_SCAN_RANGE = SPEED_LIMIT_METRES_PER_SECOND * 3.0;  // How far ahead to look for another vehicle when determining lane speeds
// static constexpr double LANE_SPEED_BACKWARD_SCAN_RANGE = SPEED_LIMIT_METRES_PER_SECOND * 0.25;  // How far behind to look for another vehicle when determining lane speeds
static constexpr double LANE_CHANGE_CLEAR = SPEED_LIMIT_METRES_PER_SECOND * 0.5;  // How far behind and ahead to look into other lanes to see if another vehicle is blocking a lane change

static constexpr unsigned int LANE_CHANGE_PENALTY = 5u;  // How many ticks to prevent the vehicle from changing lanes after doing so, or when distance to desired lane is too great
static constexpr double D_LIMIT_FOR_LANE_CHANGE_PENALTY = 0.5;  // What's the maximum allowed d distance to a lane before a lange change penalty is imposed


PathPlanner::PathPlanner(std::vector<Waypoint> waypoints) :
    m_waypoints(waypoints)
{
}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::planPath(const SimulatorResponseData& simulatorData)
{
    updateTrajectoryHistory(simulatorData);
    spdlog::trace("[planPath] EgoCar: x={}, y={}, s={}, d={}", simulatorData.egoCar.x, simulatorData.egoCar.y, simulatorData.egoCar.s, simulatorData.egoCar.d);

    // Kinematics kinematics = computeXyKinematics();

    const auto usePrevious = 5;
    Kinematics kinematics = computeXyKinematicsHist(simulatorData.prevPathX, simulatorData.prevPathY, usePrevious);
    spdlog::trace("[planPath] XY Kinematics: vx={}, ax={}, vy={}, ay={}", kinematics.velocity0, kinematics.acceleration0, kinematics.velocity1, kinematics.acceleration1);
    spdlog::trace("[planPath] Reported car speed vs kinematics dx, dy: {} vs ({}, {})", MPH_TO_METRES_PER_SECOND(simulatorData.egoCar.speed), kinematics.velocity0, kinematics.velocity1);

    const std::array<double, 3> laneSpeeds = getLaneSpeeds(simulatorData.egoCar, simulatorData.otherCars);
    spdlog::trace("[planPath] Lane Speeds: {}, {}, {}", laneSpeeds[0], laneSpeeds[1], laneSpeeds[2]);

    // Keep a counter that attempts to track when a car has completed a lane change manoeuvre
    // Side effect of low penalty tolerance: no overtaking on the curved road
    if (std::abs(simulatorData.egoCar.d - m_currentLaneD) > D_LIMIT_FOR_LANE_CHANGE_PENALTY)
    {
        m_laneChangeDelay = LANE_CHANGE_PENALTY;
        spdlog::trace("[planPath] Applied lane change penalty for d distance of {}.", std::abs(simulatorData.egoCar.d - m_currentLaneD));
    }
    else if (m_laneChangeDelay != 0)
    {
        --m_laneChangeDelay;
        spdlog::trace("[planPath] Remaining lane change penalty: {}", m_laneChangeDelay);
    }
    else
    {
        // Allow lane change
        if (m_currentLaneD == D_LEFT_LANE && laneSpeeds[1] - LANE_CHANGE_COST > laneSpeeds[0] && not isLaneBlocked(1, simulatorData.egoCar, simulatorData.otherCars))
        {
            spdlog::info("[planPath] Changing to middle lane.");
            m_currentLaneD = D_MIDDLE_LANE;
            m_currentLaneIndex = 1;
            m_laneChangeDelay = LANE_CHANGE_PENALTY;
        }
        else if (m_currentLaneD == D_RIGHT_LANE && laneSpeeds[1] - LANE_CHANGE_COST > laneSpeeds[2] && not isLaneBlocked(1, simulatorData.egoCar, simulatorData.otherCars))
        {
            spdlog::info("[planPath] Changing to middle lane.");
            m_currentLaneD = D_MIDDLE_LANE;
            m_currentLaneIndex = 1;
            m_laneChangeDelay = LANE_CHANGE_PENALTY;
        }
        else if (m_currentLaneD == D_MIDDLE_LANE && (laneSpeeds[0] - LANE_CHANGE_COST > laneSpeeds[1] || laneSpeeds[2] - LANE_CHANGE_COST > laneSpeeds[1]))
        {
            if (laneSpeeds[0] > laneSpeeds[2] && not isLaneBlocked(0, simulatorData.egoCar, simulatorData.otherCars))
            {
                spdlog::info("[planPath] Changing to left lane.");
                m_currentLaneD = D_LEFT_LANE;
                m_currentLaneIndex = 0;
                m_laneChangeDelay = LANE_CHANGE_PENALTY;
            }
            else if (not isLaneBlocked(2, simulatorData.egoCar, simulatorData.otherCars))
            {
                spdlog::info("[planPath] Changing to right lane.");
                m_currentLaneD = D_RIGHT_LANE;
                m_currentLaneIndex = 2;
                m_laneChangeDelay = LANE_CHANGE_PENALTY;
            }
        }
    }

    std::pair<std::vector<double>, std::vector<double>> xyTrajectory = genPathWithPast(
        simulatorData.egoCar, kinematics, simulatorData.prevPathX, simulatorData.prevPathY, laneSpeeds[m_currentLaneIndex], simulatorData.otherCars, usePrevious);

    spdlog::debug("[planPath] --- --- --- --- ---");
    m_prevSentTrajectoryX = xyTrajectory.first;
    m_prevSentTrajectoryY = xyTrajectory.second;
    return xyTrajectory;
}

void PathPlanner::updateTrajectoryHistory(const SimulatorResponseData& simulatorData)
{
    assert(m_prevSentTrajectoryX.size() == m_prevSentTrajectoryY.size());
    assert(simulatorData.prevPathX.size() == simulatorData.prevPathY.size());

    const int executedCommandsCount = m_prevSentTrajectoryX.size() - simulatorData.prevPathX.size();

    for (auto it = m_prevSentTrajectoryX.begin(); it != m_prevSentTrajectoryX.begin() + executedCommandsCount; ++it)
    {
        m_trajectoryHistoryX.push_front(*it);
    }
    if (m_trajectoryHistoryX.size() > TRAJECTORY_HISTORY_LENGTH)
    {
        m_trajectoryHistoryX.resize(TRAJECTORY_HISTORY_LENGTH);
    }

    for (auto it = m_prevSentTrajectoryY.begin(); it != m_prevSentTrajectoryY.begin() + executedCommandsCount; ++it)
    {
        m_trajectoryHistoryY.push_front(*it);
    }
    if (m_trajectoryHistoryY.size() > TRAJECTORY_HISTORY_LENGTH)
    {
        m_trajectoryHistoryY.resize(TRAJECTORY_HISTORY_LENGTH);
    }

    spdlog::trace("[updateTrajectoryHistory] XY history queue size is now ({}, {})", m_trajectoryHistoryX.size(), m_trajectoryHistoryY.size());
}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::genPath(
    const EgoCar& egoCar, const Kinematics& xyKinematics, const double maxLaneSpeed, const std::vector<OtherCar>& otherCars)
{
    // Account for max acceleration
    spdlog::trace("[genPath] EgoCar speed, max lane speed: {}, {}", MPH_TO_METRES_PER_SECOND(egoCar.speed), maxLaneSpeed);
    double maxSpeed = std::min(maxLaneSpeed, MPH_TO_METRES_PER_SECOND(egoCar.speed) + MAX_ACCELERATION);

    // Estimate target S point, as well as xy speed vector
    double nextS = egoCar.s + maxSpeed * PATH_DURATION_SECONDS;
    double auxS = nextS - 0.1;  // Hard to tell if false positive, but setting higher values here might prevent some freakouts (though mostly not)

    double nextX, nextY;
    std::tie(nextX, nextY) = getXY(nextS, m_currentLaneD, m_waypoints);

    double auxX, auxY;
    std::tie(auxX, auxY) = getXY(auxS, m_currentLaneD, m_waypoints);

    double velocityProportionX = (nextX - auxX) / distance(nextX, nextY, auxX, auxY);
    double velocityProportionY = (nextY - auxY) / distance(nextX, nextY, auxX, auxY);


    // TODO: create kinematics based on returned path remainder. Then start from there, using at least the first 4 points on that path. Maybe this will make it smooth.

    // Generate trajectories
    const std::array<double, 6> xParams = polynomialTrajectoryParameters(PATH_DURATION_SECONDS,
        egoCar.x, xyKinematics.velocity0, xyKinematics.acceleration0,
        nextX, maxSpeed * velocityProportionX, 0.0);

    const std::array<double, 6> yParams = polynomialTrajectoryParameters(PATH_DURATION_SECONDS,
        egoCar.y, xyKinematics.velocity1, xyKinematics.acceleration1,
        nextY, maxSpeed * velocityProportionY, 0.0);

    std::vector<double> xTrajectory = generateTrajectoryFromParams(PATH_DURATION_SECONDS, NODE_TRAVERSAL_RATE_SECONDS, xParams);
    std::vector<double> yTrajectory = generateTrajectoryFromParams(PATH_DURATION_SECONDS, NODE_TRAVERSAL_RATE_SECONDS, yParams);

    // // Check XY->SD-XY conversion, for the fun of it
    // std::vector<double> sTrajectory, dTrajectory, xTraj0, yTraj0;
    // std::tie(sTrajectory, dTrajectory) = getFrenet(xTrajectory, yTrajectory, m_waypoints);
    // std::tie(xTraj0, yTraj0) = getXY(sTrajectory, dTrajectory, m_waypoints);

    // std::cout << "Trajectory checks (x,y)->(s,d)->(x,y):" << std::endl;
    // for (int i = 0; i < sTrajectory.size(); ++i)
    // {
    //     std::cout << "(" << xTrajectory[i] << ", " << yTrajectory[i] << ") -> "
    //                 << "(" << sTrajectory[i] << ", " << dTrajectory[i] << ") -> "
    //                 << "(" << xTraj0[i]      << ", " << yTraj0[i]      << ")" << std::endl;
    // }
    // std::cout << "Car S, D:" << egoCar.s << ", " << egoCar.d << std::endl;

    return std::make_pair(xTrajectory, yTrajectory);
}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::genPathWithPast(
    const EgoCar& egoCar, const Kinematics& xyKinematics, const std::vector<double>& prevPathX, const std::vector<double>& prevPathY,
    const double maxLaneSpeed, const std::vector<OtherCar>& otherCars, const unsigned pastElementCount)
{
    assert(prevPathX.size() == prevPathY.size());
    int usePrevious = std::min((unsigned)prevPathX.size(), pastElementCount);

    double startX, startY, startS, speed;
    if (usePrevious <= 2)
    {
        usePrevious = 0;  // Ignore history

        startX = egoCar.x;
        startY = egoCar.y;
        startS = egoCar.s;
        speed = MPH_TO_METRES_PER_SECOND(egoCar.speed);
    }
    else
    {
        std::vector<double> prevPathS, prevPathD;
        std::tie(prevPathS, prevPathD) = getFrenet(prevPathX, prevPathY, m_waypoints);

        startX = prevPathX[usePrevious-1];
        startY = prevPathY[usePrevious-1];
        startS = prevPathS[usePrevious-1];
        speed = prevPathS[usePrevious-1] - prevPathS[usePrevious-2];
    }

    spdlog::debug("[genPath] speed={}, maxLaneSpeed={}", speed, maxLaneSpeed);
    double maxSpeed = std::min(maxLaneSpeed, speed + MAX_ACCELERATION);

    // Estimate tar[genPath] get S point, as well as xy speed vector
    spdlog::debug("[genPath] maxSpeed={}, startS={}, usePrevious={}", maxSpeed, startS, usePrevious);
    double endS = startS + maxSpeed * (PATH_DURATION_SECONDS - (usePrevious - 1) * NODE_TRAVERSAL_RATE_SECONDS);
    double auxS = endS - 0.1;

    spdlog::debug("[genPath] endS={}, m_currentLaneD={}", endS, m_currentLaneD);

    double endX, endY;
    std::tie(endX, endY) = getXY(endS, m_currentLaneD, m_waypoints);

    spdlog::debug("[genPath] auxS={}", auxS);

    double auxX, auxY;
    std::tie(auxX, auxY) = getXY(auxS, m_currentLaneD, m_waypoints);

    double velocityProportionX = (endX - auxX) / distance(endX, endY, auxX, auxY);
    double velocityProportionY = (endY - auxY) / distance(endX, endY, auxX, auxY);

    // Generate trajectories
    const std::array<double, 6> xParams = polynomialTrajectoryParameters(PATH_DURATION_SECONDS,
        startX, xyKinematics.velocity0, xyKinematics.acceleration0,
        endX, maxSpeed * velocityProportionX, 0.0);

    const std::array<double, 6> yParams = polynomialTrajectoryParameters(PATH_DURATION_SECONDS,
        startY, xyKinematics.velocity1, xyKinematics.acceleration1,
        endY, maxSpeed * velocityProportionY, 0.0);

    spdlog::debug("[genPath] endX={}, endY={}", endX, endY);

    std::vector<double> xTrajectory = generateTrajectoryFromParams(PATH_DURATION_SECONDS, NODE_TRAVERSAL_RATE_SECONDS, xParams);
    std::vector<double> yTrajectory = generateTrajectoryFromParams(PATH_DURATION_SECONDS, NODE_TRAVERSAL_RATE_SECONDS, yParams);

    // Add prev path to path
    spdlog::debug("[genPath] PrevPath size: {}, use prev: {}", prevPathX.size(), usePrevious);

    xTrajectory.insert(xTrajectory.begin(), prevPathX.begin(), prevPathX.begin() + usePrevious);
    yTrajectory.insert(yTrajectory.begin(), prevPathY.begin(), prevPathY.begin() + usePrevious);

    return std::make_pair(xTrajectory, yTrajectory);
}

Kinematics PathPlanner::computeSdKinematics()
{
    if (m_trajectoryHistoryX.size() < 4)
    {
        // Just started

        // Need at least 2 for velocity, 3 for accelearation
        // and 1 extra to all for previous state when computing heading for getFrenet

        return Kinematics {};
    }

    auto xHistIter = m_trajectoryHistoryX.begin();
    auto yHistIter = m_trajectoryHistoryY.begin();

    double heading0 = orientation(*(yHistIter + 0) - *(yHistIter + 1), *(xHistIter + 0) - *(xHistIter + 1));
    double heading1 = orientation(*(yHistIter + 1) - *(yHistIter + 2), *(xHistIter + 1) - *(xHistIter + 2));
    double heading2 = orientation(*(yHistIter + 2) - *(yHistIter + 3), *(xHistIter + 2) - *(xHistIter + 3));

    double s0, s1, s2, d0, d1, d2;
    std::tie(s0, d0) = getFrenet(*(xHistIter + 0), *(yHistIter + 0), heading0, m_waypoints);
    std::tie(s1, d1) = getFrenet(*(xHistIter + 1), *(yHistIter + 1), heading1, m_waypoints);
    std::tie(s2, d2) = getFrenet(*(xHistIter + 2), *(yHistIter + 2), heading2, m_waypoints);

    spdlog::trace("[computeSdKinematics] Current frenet coordinates: - 0:({}, {}), 1:({}, {}), 2:({}, {})", s0, d0, s1, d1, s2, d2);

    double sVelocity1 = (s0 - s1) / NODE_TRAVERSAL_RATE_SECONDS;
    double sVelocity2 = (s1 - s2) / NODE_TRAVERSAL_RATE_SECONDS;
    double dVelocity1 = (d0 - d1) / NODE_TRAVERSAL_RATE_SECONDS;
    double dVelocity2 = (d1 - d2) / NODE_TRAVERSAL_RATE_SECONDS;

    double sAcceleration = (sVelocity1 - sVelocity2) / NODE_TRAVERSAL_RATE_SECONDS;
    double dAcceleration = (dVelocity1 - dVelocity2) / NODE_TRAVERSAL_RATE_SECONDS;

    return { sVelocity1, sAcceleration, dVelocity1, dAcceleration };
}

Kinematics PathPlanner::computeXyKinematics()
{
    assert(m_trajectoryHistoryX.size() == m_trajectoryHistoryY.size());
    if (m_trajectoryHistoryX.size() < 4)
    {
        // Just started. The history will fill up on later ticks.

        // Need at least 2 for velocity, 3 for accelearation
        // and 1 extra to all for previous state when computing heading for getFrenet

        return Kinematics {};
    }

    auto xHistIter = m_trajectoryHistoryX.begin();
    auto yHistIter = m_trajectoryHistoryY.begin();

    double xVelocity1 = (*(xHistIter + 0) - *(xHistIter + 1)) / NODE_TRAVERSAL_RATE_SECONDS;
    double xVelocity2 = (*(xHistIter + 1) - *(xHistIter + 2)) / NODE_TRAVERSAL_RATE_SECONDS;
    double yVelocity1 = (*(yHistIter + 0) - *(yHistIter + 1)) / NODE_TRAVERSAL_RATE_SECONDS;
    double yVelocity2 = (*(yHistIter + 1) - *(yHistIter + 2)) / NODE_TRAVERSAL_RATE_SECONDS;

    double xAcceleration = (xVelocity1 - xVelocity2) / NODE_TRAVERSAL_RATE_SECONDS;
    double yAcceleration = (yVelocity1 - yVelocity2) / NODE_TRAVERSAL_RATE_SECONDS;

    return { xVelocity1, xAcceleration, yVelocity1, yAcceleration };
}

Kinematics PathPlanner::computeXyKinematicsHist(std::vector<double> prevPathX, std::vector<double> prevPathY, const unsigned pastElementCount)
{
    assert(prevPathX.size() == prevPathY.size());
    assert(m_trajectoryHistoryX.size() == m_trajectoryHistoryY.size());

    prevPathX.resize(pastElementCount);
    prevPathY.resize(pastElementCount);

    if (prevPathX.size() + m_trajectoryHistoryX.size() < 3)
    {
        // Just started. The history will fill up on later ticks.

        // Need at least 2 for velocity, 3 for accelearation
        // and 1 extra to all for previous state when computing heading for getFrenet

        return Kinematics {};
    }

    // Basic idea: use elements from prevPath to compute kinematics, but borrow from history if not enough.
    auto xHistIter = m_trajectoryHistoryX.begin();
    auto yHistIter = m_trajectoryHistoryY.begin();

    while (prevPathX.size() < 3)
    {
        prevPathX.insert(prevPathX.begin(), *xHistIter);
        xHistIter++;

        prevPathY.insert(prevPathY.begin(), *yHistIter);
        yHistIter++;
    }

    double xVelocity1 = (prevPathX[2] - prevPathX[1]) / NODE_TRAVERSAL_RATE_SECONDS;
    double xVelocity2 = (prevPathX[1] - prevPathX[0]) / NODE_TRAVERSAL_RATE_SECONDS;
    double yVelocity1 = (prevPathY[2] - prevPathY[1]) / NODE_TRAVERSAL_RATE_SECONDS;
    double yVelocity2 = (prevPathY[1] - prevPathY[0]) / NODE_TRAVERSAL_RATE_SECONDS;

    double xAcceleration = (xVelocity1 - xVelocity2) / NODE_TRAVERSAL_RATE_SECONDS;
    double yAcceleration = (yVelocity1 - yVelocity2) / NODE_TRAVERSAL_RATE_SECONDS;

    return { xVelocity1, xAcceleration, yVelocity1, yAcceleration };
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

std::array<double, 6> PathPlanner::polynomialTrajectoryParameters(
    double totalTime, double startPos, double startSpeed, double startAcc, double endPos, double endSpeed, double endAcc)
{
    spdlog::trace("[polynomialTrajectoryParameters] Start params: ({}, {}, {}), End params: ({}, {}, {})", totalTime, startPos, startSpeed, startAcc, endPos, endSpeed, endAcc);

    Eigen::Matrix3d params;
    params <<     pow(totalTime, 3),      pow(totalTime, 4),      pow(totalTime, 5),
              3 * pow(totalTime, 2), 4  * pow(totalTime, 3), 5  * pow(totalTime, 4),
              6 * pow(totalTime, 1), 12 * pow(totalTime, 2), 20 * pow(totalTime, 3);

    Eigen::Vector3d res;
    res << endPos - (startPos + startSpeed * totalTime + 0.5 * startAcc * pow(totalTime, 2)),
           endSpeed - (startSpeed + startAcc * totalTime),
           endAcc - startAcc;

    Eigen::Vector3d result = params.inverse() * res;

    return { startPos, startSpeed, 0.5 * startAcc, result[0], result[1], result[2] };
}

std::vector<double> PathPlanner::generateTrajectoryFromParams(double totalTime, double timeIncrement, const std::array<double, 6>& polyParams)
{
    std::vector<double> results;
    int resultCount = totalTime / timeIncrement;
    results.reserve(resultCount);

    for (int i = 1; i <= resultCount; ++i)
    {
        double t = timeIncrement * i;
        double val = polyParams[0] * std::pow(t, 0) + polyParams[1] * std::pow(t, 1) + polyParams[2] * std::pow(t, 2) +
                     polyParams[3] * std::pow(t, 3) + polyParams[4] * std::pow(t, 4) + polyParams[5] * std::pow(t, 5);
        results.push_back(val);
    }

    return results;
}

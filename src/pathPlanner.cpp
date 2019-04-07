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
static_assert(TRAJECTORY_HISTORY_LENGTH >= 4);  // Need at least this many to compute kinematics

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

    Kinematics kinematics = computeXyKinematics();

    // const auto usePrevious = 5;
    // Kinematics kinematics = computeXyKinematicsHist(simulatorData.prevPathX, simulatorData.prevPathY, usePrevious);

    spdlog::trace("[planPath] XY Kinematics: vx={}, ax={}, vy={}, ay={}", kinematics.velocity0, kinematics.acceleration0, kinematics.velocity1, kinematics.acceleration1);
    spdlog::trace("[planPath] Reported car speed vs kinematics dx, dy: {} vs ({}, {})", MPH_TO_METRES_PER_SECOND(simulatorData.egoCar.speed), kinematics.velocity0, kinematics.velocity1);

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

    // std::pair<std::vector<double>, std::vector<double>> xyTrajectory = genPath(simulatorData.egoCar, kinematics, laneSpeeds[m_targetLaneIndex], simulatorData.otherCars);
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
    std::tie(nextX, nextY) = getXY(nextS, m_targetLaneD, m_waypoints);

    double auxX, auxY;
    std::tie(auxX, auxY) = getXY(auxS, m_targetLaneD, m_waypoints);

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

    double startX, startY, startS, startD, yaw, speed;
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
        speed = prevPathS[usePrevious-1] - prevPathS[usePrevious-2];  // TODO: this needs to be divided by NODE_TRAVERSAL_RATE_SECONDS
    }

    spdlog::debug("[genPath] speed={}, maxLaneSpeed={}", speed, maxLaneSpeed);
    double maxSpeed = std::min(maxLaneSpeed, speed + MAX_ACCELERATION);

    // Estimate tar[genPath] get S point, as well as xy speed vector
    spdlog::debug("[genPath] maxSpeed={}, startS={}, usePrevious={}", maxSpeed, startS, usePrevious);
    double endS = startS + maxSpeed * (PATH_DURATION_SECONDS - (usePrevious - 1) * NODE_TRAVERSAL_RATE_SECONDS);
    double auxS = endS - 0.1;

    spdlog::debug("[genPath] endS={}, m_targetLaneD={}", endS, m_targetLaneD);

    double endX, endY;
    std::tie(endX, endY) = getXY(endS, m_targetLaneD, m_waypoints);

    spdlog::debug("[genPath] auxS={}", auxS);

    double auxX, auxY;
    std::tie(auxX, auxY) = getXY(auxS, m_targetLaneD, m_waypoints);

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

std::pair<std::vector<double>, std::vector<double>> PathPlanner::genPathSpline(
    const EgoCar& egoCar, const double maxLaneSpeed, const std::vector<double>& prevPathX, const std::vector<double>& prevPathY, const unsigned keepPrevious)
{
    // Because of the issues in getFrenet function, I will not fully trust the frenet coordinates and will try to avoid them if I can.
    // Still, we need them to compute the destination, so they will be used in 2 cases:
    // 1. The initial s position of the vehicle. This does not use getFrenet, but obtains it from the simulator, with hopefully better accuracy.
    // 2. The goal s position for the vehicle. If the coordinate is incorrect, the goal will be far enough to be corrected at a later timepoint.

    assert(prevPathX.size() == prevPathY.size());
    const unsigned prevAvailable = std::min((unsigned)prevPathX.size(), keepPrevious);

    std::cout << "Prev path:" << std::endl;
    std::cout << "X:  " << toString(prevPathX) << std::endl << std::endl;
    std::cout << "Y:  " << toString(prevPathY) << std::endl << std::endl;

    // Set up spline keypoints
    spdlog::debug("[genPathSpline] maxLaneSpeed = {}, startSpeed + MAX_ACCELERATION = {}", maxLaneSpeed, MPH_TO_METRES_PER_SECOND(egoCar.speed) + MAX_ACCELERATION);
    const double maxSpeed = std::min(maxLaneSpeed, MPH_TO_METRES_PER_SECOND(egoCar.speed) + MAX_ACCELERATION);

    // double nextS = egoCar.s + maxSpeed * PATH_DURATION_SECONDS * 0.25;
    // double nextD = egoCar.d + dDifference * 0.25;

    // Just because spline needs > 2 points
    const double dDifference = m_targetLaneD - egoCar.d;
    double beforeEndS = egoCar.s + maxSpeed * PATH_DURATION_SECONDS * 0.5;
    double beforeEndD = egoCar.d + dDifference * 0.5;

    double endS = egoCar.s + maxSpeed * PATH_DURATION_SECONDS;
    double endD = m_targetLaneD;

    // const std::vector<double> sPoints = { egoCar.s, nextS, beforeEndS, endS };
    // const std::vector<double> dPoints = { egoCar.d, nextD, beforeEndD, endD };

    // std::cout << "spline sPoints:  " << toString(sPoints) << std::endl << std::endl;
    // std::cout << "spline dPoints:  " << toString(dPoints) << std::endl << std::endl;


    // std::vector<double> xPoints, yPoints;
    // std::tie(xPoints, yPoints) = getXY(sPoints, dPoints, m_waypoints);

    // std::cout << "spline xPoints:  " << toString(xPoints) << std::endl << std::endl;
    // std::cout << "spline yPoints:  " << toString(yPoints) << std::endl << std::endl;

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
        spdlog::debug("Considering historical ({}, {}) to prepend to path. Distance to reference = {}.", histX, histY, distToRef);
        if (distToRef > 3.0)
        {
            spdlog::info("Prepended.");
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

    std::cout << "xPoints:  " << toString(xPoints) << std::endl << std::endl;
    std::cout << "yPoints:  " << toString(yPoints) << std::endl << std::endl;

    std::vector<double> xCarPoints, yCarPoints;
    std::tie(xCarPoints, yCarPoints) = worldCoordToCarCoord(egoCar, xPoints, yPoints);

    std::cout << "local car xPoints:  " << toString(xCarPoints) << std::endl << std::endl;
    std::cout << "local car yPoints:  " << toString(yCarPoints) << std::endl << std::endl;

    // Example xCarPoints: 0.434606 0.869412 1.30431 1.73945 2.17479 2.61027 3.04584 3.48166 3.91767 4.35386 2.4132 -0.30777 -1.66825
    // Therefore, it still seems that the issue happens when joining in the new points / adding new s

    tk::spline spl;
    if (not std::is_sorted(xCarPoints.begin(), xCarPoints.end()))
    {
        spdlog::error("[genPathSpline] xCarPoints are not sorted: {}", toString(xCarPoints));
        assert(false);
    }
    spl.set_points(xCarPoints, yCarPoints);

    // Compute acceleration that helps reach destination in given time; set x, y
    const double carEndX = xCarPoints[xCarPoints.size() - 1];
    xCarPoints.clear();
    yCarPoints.clear();

    // Derived from the standard  x = x0 + v * t + a * t^2 / 2, with x0 = 0
    const double acceleration = 2 * (carEndX - egoCar.speed * PATH_DURATION_SECONDS) / std::pow(PATH_DURATION_SECONDS, 2.0);
    for (double t = NODE_TRAVERSAL_RATE_SECONDS; t < PATH_DURATION_SECONDS; t += NODE_TRAVERSAL_RATE_SECONDS)
    {
        const double x = MPH_TO_METRES_PER_SECOND(egoCar.speed) * t + acceleration * std::pow(t, 2.0) / 2.0;
        xCarPoints.push_back(x);
        yCarPoints.push_back(spl(x));
    }

    std::tie(xPoints, yPoints) = carCoordToWorldCoord(egoCar, xCarPoints, yCarPoints);

    std::cout << "Sending path X:  " << toString(xPoints) << std::endl << std::endl;
    std::cout << "Sending path Y:  " << toString(yPoints) << std::endl << std::endl;

    // std::vector<double> sPointsNew, dPointsNew;
    // std::tie(sPointsNew, dPointsNew) = getFrenet(xPoints, yPoints, m_waypoints);

    // std::cout << "Sending path S:  " << toString(sPointsNew) << std::endl << std::endl;
    // std::cout << "Sending path D:  " << toString(dPointsNew) << std::endl << std::endl;

    return std::make_pair(xPoints, yPoints);
}

Kinematics PathPlanner::computeSdKinematics()
{
    if (m_historyEgoX.size() < 4)
    {
        // Just started

        // Need at least 2 for velocity, 3 for accelearation
        // and 1 extra to all for previous state when computing heading for getFrenet

        return Kinematics {};
    }

    auto xHistIter = m_historyEgoX.begin();
    auto yHistIter = m_historyEgoY.begin();

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
    assert(m_historyEgoX.size() == m_historyEgoY.size());
    if (m_historyEgoX.size() < 4)
    {
        // Just started. The history will fill up on later ticks.

        // Need at least 2 for velocity, 3 for accelearation
        // and 1 extra to all for previous state when computing heading for getFrenet

        return Kinematics {};
    }

    auto xHistIter = m_historyEgoX.begin();
    auto yHistIter = m_historyEgoY.begin();

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
    assert(m_historyEgoX.size() == m_historyEgoY.size());

    prevPathX.resize(pastElementCount);
    prevPathY.resize(pastElementCount);

    if (prevPathX.size() + m_historyEgoX.size() < 3)
    {
        // Just started. The history will fill up on later ticks.

        // Need at least 2 for velocity, 3 for accelearation
        // and 1 extra to all for previous state when computing heading for getFrenet

        return Kinematics {};
    }

    // Basic idea: use elements from prevPath to compute kinematics, but borrow from history if not enough.
    auto xHistIter = m_historyEgoX.begin();
    auto yHistIter = m_historyEgoY.begin();

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

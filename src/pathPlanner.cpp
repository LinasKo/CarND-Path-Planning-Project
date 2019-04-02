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
    spdlog::debug("[planPath] EgoCar: x={}, y={}, s={}, d={}", simulatorData.egoCar.x, simulatorData.egoCar.y, simulatorData.egoCar.s, simulatorData.egoCar.d);

    Kinematics kinematics = computeXyKinematics();
    // Kinematics kinematics = computeXyKinematicsAvg();  // DOES NOT WORK - average velocity relies on previous state too much
    spdlog::debug("[planPath] XY Kinematics: vx={}, ax={}, vy={}, ay={}", kinematics.velocity0, kinematics.acceleration0, kinematics.velocity1, kinematics.acceleration1);

    const std::array<double, 3> laneSpeeds = getLaneSpeeds(simulatorData.egoCar, simulatorData.otherCars);
    spdlog::trace("[planPath] Lane Speeds: {}, {}, {}", laneSpeeds[0], laneSpeeds[1], laneSpeeds[2]);

    // Keep a counter that attempts to track when a car has completed a lane change manoeuvre
    // Side effect of low penalty tolerance: no overtaking on the curved road :)
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

    std::pair<std::vector<double>, std::vector<double>> xyTrajectory = genPath(simulatorData.egoCar, kinematics, laneSpeeds[m_currentLaneIndex], simulatorData.otherCars);

    // Bring path closer to center of lane
    // std::pair<std::vector<double>, std::vector<double>> xySmooth = smoothenPath(simulatorData.egoCar.yaw, xyTrajectory.first, xyTrajectory.second, 0.0);
    auto xySmooth = xyTrajectory;
    spdlog::trace("[planPath] Lengths of smoothened path: {}, {}", xySmooth.first.size(), xySmooth.second.size());

    m_prevSentTrajectoryX = xySmooth.first;
    m_prevSentTrajectoryY = xySmooth.second;

    spdlog::debug("[planPath] --- --- --- --- ---");
    return xySmooth;

    // TODO: looking at the logs, it seems that maybe when the car goes out of control, it is because the d value cannot stabilize at the lane centre and just fluctuates wildly, affecting velocity / acceleration.
    /*
     * At the same time, it actually doesn't look like that, so I don't know what to think. The car can just spin out of control out of the blue, while going straight. Maybe I should watch the tail of the planned trajectory.
     *
            [2019-04-02 22:42:52.164] [debug] [planPath] XY Kinematics: vx=16.2833, ax=-0.66847, vy=2.84658, ay=0.775625
            [2019-04-02 22:42:52.164] [debug] [planPath] --- --- --- --- ---
            [2019-04-02 22:42:52.220] [debug] [planPath] EgoCar: x=1880.44, y=1147.46, s=1106.38, d=5.49663
            [2019-04-02 22:42:52.220] [debug] [planPath] XY Kinematics: vx=16.2545, ax=-0.523559, vy=2.88454, ay=0.749856
            [2019-04-02 22:42:52.220] [debug] [planPath] --- --- --- --- ---
            [2019-04-02 22:42:52.272] [debug] [planPath] EgoCar: x=1881.09, y=1147.57, s=1107.04, d=5.5238
            [2019-04-02 22:42:52.272] [debug] [planPath] XY Kinematics: vx=16.2395, ax=-5.68126, vy=2.9066, ay=3.64966
            [2019-04-02 22:42:52.272] [debug] [planPath] --- --- --- --- ---
            [2019-04-02 22:42:52.328] [debug] [planPath] EgoCar: x=1882.07, y=1147.75, s=1108.03, d=5.56557
            [2019-04-02 22:42:52.328] [debug] [planPath] XY Kinematics: vx=15.9803, ax=-4.90123, vy=3.07518, ay=3.21321
            [2019-04-02 22:42:52.328] [debug] [planPath] --- --- --- --- ---
            [2019-04-02 22:42:52.380] [debug] [planPath] EgoCar: x=1883, y=1147.95, s=1108.98, d=5.57787
            [2019-04-02 22:42:52.380] [debug] [planPath] XY Kinematics: vx=15.7583, ax=-4.17558, vy=3.22307, ay=2.81274
            [2019-04-02 22:42:52.380] [debug] [planPath] --- --- --- --- ---
            [2019-04-02 22:42:52.432] [debug] [planPath] EgoCar: x=1883.64, y=1148.07, s=1109.63, d=5.59499
            [2019-04-02 22:42:52.432] [debug] [planPath] XY Kinematics: vx=15.641, ax=27.7992, vy=3.30307, ay=-15.0491
            [2019-04-02 22:42:52.432] [debug] [planPath] --- --- --- --- ---
            [2019-04-02 22:42:52.484] [debug] [planPath] EgoCar: x=1884.56, y=1148.28, s=1110.58, d=5.59072
            [2019-04-02 22:42:52.484] [debug] [planPath] XY Kinematics: vx=16.9113, ax=24.0427, vy=2.61773, ay=-12.9423
            [2019-04-02 22:42:52.484] [debug] [planPath] --- --- --- --- ---
            [2019-04-02 22:42:52.544] [debug] [planPath] EgoCar: x=1885.69, y=1148.37, s=1111.7, d=5.74636
            [2019-04-02 22:42:52.544] [debug] [planPath] XY Kinematics: vx=18.0015, ax=20.5281, vy=2.03338, ay=-10.9724


     * Note that the example below might indicate that acceleration change is a problem after all.

            [2019-04-02 23:23:56.372] [debug] [genPath] Changes in velocity along (x, y): (-1.67965, -0.574535)
            [2019-04-02 23:23:56.372] [debug] [genPath] Changes in acceleration along (x, y): (-4.46693, -1.86165)
            [2019-04-02 23:23:56.372] [debug] [planPath] --- --- --- --- ---
            [2019-04-02 23:23:56.424] [debug] [genPath] Changes in velocity along (x, y): (-1.87153, -0.655511)
            [2019-04-02 23:23:56.424] [debug] [genPath] Changes in acceleration along (x, y): (-3.47994, -1.48188)
            [2019-04-02 23:23:56.424] [debug] [planPath] --- --- --- --- ---
            [2019-04-02 23:23:56.476] [debug] [genPath] Changes in velocity along (x, y): (-1.96517, -0.695891)
            [2019-04-02 23:23:56.476] [debug] [genPath] Changes in acceleration along (x, y): (-46.3932, -16.0834)
            [2019-04-02 23:23:56.476] [debug] [planPath] --- --- --- --- ---
            [2019-04-02 23:23:56.532] [debug] [genPath] Changes in velocity along (x, y): (-4.06455, -1.42413)
            [2019-04-02 23:23:56.532] [debug] [genPath] Changes in acceleration along (x, y): (-39.4798, -13.7004)
     *
     */

    // At any rate, it seems to happen mostly when the car attempts to turn at high speed. Need to reduce speed on turns.
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
    double auxS = nextS - 0.01;

    double nextX, nextY;
    std::tie(nextX, nextY) = getXY(nextS, m_currentLaneD, m_waypoints);

    double auxX, auxY;
    std::tie(auxX, auxY) = getXY(auxS, m_currentLaneD, m_waypoints);

    double velocityProportionX = (nextX - auxX) / distance(nextX, nextY, auxX, auxY);
    double velocityProportionY = (nextY - auxY) / distance(nextX, nextY, auxX, auxY);

    // Attempting to tackle vehicle going out of control by reducing max speed when velocity direction is changing
    spdlog::debug("[genPath] Changes in velocity along (x, y): ({}, {})", maxSpeed * velocityProportionX - xyKinematics.velocity0, maxSpeed * velocityProportionY - xyKinematics.velocity1);
    spdlog::debug("[genPath] Changes in acceleration along (x, y): ({}, {})", -xyKinematics.acceleration0, -xyKinematics.acceleration1);
    double velocityDampenRatio = 1.0;
    double xVelocityChange = maxSpeed * velocityProportionX - xyKinematics.velocity0;
    double yVelocityChange = maxSpeed * velocityProportionY - xyKinematics.velocity1;
    const double velocityChange = std::abs(xVelocityChange) + std::abs(yVelocityChange);
    if (velocityChange != 0 && velocityChange > MAX_VELOCITY_CHANGE)
    {
        velocityDampenRatio = MAX_VELOCITY_CHANGE / velocityChange;

        if (velocityDampenRatio != 1.0)
        {
            // Recompute target
            double nextS = egoCar.s + maxSpeed * PATH_DURATION_SECONDS * velocityDampenRatio;
            double auxS = nextS - 0.01;

            double nextX, nextY;
            std::tie(nextX, nextY) = getXY(nextS, m_currentLaneD, m_waypoints);

            double auxX, auxY;
            std::tie(auxX, auxY) = getXY(auxS, m_currentLaneD, m_waypoints);

            double velocityProportionX = (nextX - auxX) / distance(nextX, nextY, auxX, auxY);
            double velocityProportionY = (nextY - auxY) / distance(nextX, nextY, auxX, auxY);
        }
    }

    // Generate trajectories
    const std::array<double, 6> xParams = polynomialTrajectoryParameters(PATH_DURATION_SECONDS,
        egoCar.x, xyKinematics.velocity0, xyKinematics.acceleration0,
        nextX, maxSpeed * velocityProportionX, 0.0);

    const std::array<double, 6> yParams = polynomialTrajectoryParameters(PATH_DURATION_SECONDS,
        egoCar.y, xyKinematics.velocity1, xyKinematics.acceleration1,
        nextY, maxSpeed * velocityProportionY, 0.0);

    std::vector<double> xTrajectory = generateTrajectoryFromParams(PATH_DURATION_SECONDS, NODE_TRAVERSAL_RATE_SECONDS, xParams);
    std::vector<double> yTrajectory = generateTrajectoryFromParams(PATH_DURATION_SECONDS, NODE_TRAVERSAL_RATE_SECONDS, yParams);

    return std::make_pair(xTrajectory, yTrajectory);
}

// TODO: I don't know why, but it does not recover the orgininal coordinates if xy are passed in, even without modifications
// std::pair<std::vector<double>, std::vector<double>> PathPlanner::smoothenPath(
//     const double currentHeading, std::vector<double> xPath, std::vector<double> yPath, const double smoothingStrength)
// {
//     assert(xPath.size() == yPath.size());
//     assert(smoothingStrength >= 0.0 && smoothingStrength <= 1.0);

//     for (auto i = 1; i < xPath.size(); ++i)
//     {
//         double s, d;
//         if (i == 0)
//         {
//             std::tie(s, d) = getFrenet(xPath[i], yPath[i], currentHeading, m_waypoints);
//         }
//         else
//         {
//             std::tie(s, d) = getFrenet(xPath[i], yPath[i], orientation(yPath[i] - yPath[i-1], xPath[i] - xPath[i-1]), m_waypoints);
//         }

//         // Smoothen
//         // double distToLaneCenter = m_currentLaneD - d;
//         // d += distToLaneCenter * smoothingStrength;

//         double newX, newY;
//         std::tie(newX, newY) = getXY(s, d, m_waypoints);

//         spdlog::debug("[smoothenPath] xy before: ({}, {})", xPath[i], yPath[i]);
//         spdlog::debug("[smoothenPath] xy after:  ({}, {})", newX, newY);

//         xPath[i] = newX;
//         yPath[i] = newY;
//     }

//     return std::make_pair(xPath, yPath);
// }

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

Kinematics PathPlanner::computeXyKinematicsAvg()
{
    std:assert(m_trajectoryHistoryX.size() == m_trajectoryHistoryY.size());
    const auto historySize = m_trajectoryHistoryX.size();

    if (historySize < 4)
    {
        // Just started. The history will fill up on later ticks.

        // Need at least 2 for velocity, 3 for accelearation
        // and 1 extra to all for previous state when computing heading for getFrenet

        return Kinematics {};
    }

    std::vector<double> xVelocities(historySize - 1);
    std::vector<double> yVelocities(historySize - 1);
    std::vector<double> xAccelerations(historySize - 2);
    std::vector<double> yAccelerations(historySize - 2);

    auto xHistIter = m_trajectoryHistoryX.begin();
    auto yHistIter = m_trajectoryHistoryY.begin();
    for (int i = 0; i < historySize - 1; ++i)
    {
        xVelocities[i] = (*(xHistIter + i) - *(xHistIter + i + 1)) / NODE_TRAVERSAL_RATE_SECONDS;
        yVelocities[i] = (*(yHistIter + i) - *(yHistIter + i + 1)) / NODE_TRAVERSAL_RATE_SECONDS;
    }

    for (int i = 0; i < historySize - 2; ++i)
    {
        xAccelerations[i] = (xVelocities[i] - xVelocities[i+1]) / NODE_TRAVERSAL_RATE_SECONDS;
        yAccelerations[i] = (yVelocities[i] - yVelocities[i+1]) / NODE_TRAVERSAL_RATE_SECONDS;
    }

    double meanVelocityX = std::accumulate(xVelocities.begin(), xVelocities.end(), 0.0) / xVelocities.size();
    double meanVelocityY = std::accumulate(yVelocities.begin(), yVelocities.end(), 0.0) / yVelocities.size();
    double meanAccelerationX = std::accumulate(xAccelerations.begin(), xAccelerations.end(), 0.0) / xAccelerations.size();
    double meanAccelerationY = std::accumulate(yAccelerations.begin(), yAccelerations.end(), 0.0) / yAccelerations.size();

    return { meanVelocityX, meanAccelerationX, meanVelocityY, meanAccelerationY };
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

#include "pathPlanner.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include "spdlog/spdlog.h"

#include "helpers.h"


using namespace path_planning;

static constexpr double LANE_CHANGE_COST = 1.0;  // penalty to speed when deciding whether to change to a faster lane

static constexpr double KEEP_LANE_DURATION_SECONDS = 2.5;
static constexpr double CHANGE_LANE_DURATION_SECONDS = 5.0;
static constexpr double NODE_TRAVERSAL_RATE_SECONDS = 0.02;

static constexpr double SAFETY_DISTANCE_TO_OTHER_CAR = 5.0;
static constexpr double MPH_TO_METRES_PER_SECOND(const double mph)
{
    return mph / 3600.0 * 1609.344;
}
static constexpr double SPEED_LIMIT_METRES_PER_SECOND = MPH_TO_METRES_PER_SECOND(50.0 * 0.9);  // Simulator coordinates == metres.
static constexpr double MAX_ACCELERATION = 10.0 * 0.5;

static constexpr unsigned TRAJECTORY_HISTORY_LENGTH = 5u;
static_assert(TRAJECTORY_HISTORY_LENGTH >= 4);

static constexpr double LANE_SPEED_FORWARD_SCAN_RANGE = SPEED_LIMIT_METRES_PER_SECOND * 3.0;


PathPlanner::PathPlanner(std::vector<Waypoint> waypoints) :
    m_waypoints(waypoints)
{
}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::planPath(const SimulatorResponseData& simulatorData)
{
    updateTrajectoryHistory(simulatorData);
    Kinematics kinematics = computeXyKinematics();
    spdlog::trace("[planPath] Kinematics: {}, {}, {}, {}", kinematics.velocity0, kinematics.acceleration0, kinematics.velocity1, kinematics.acceleration1);

    const std::array<double, 3> laneSpeeds = getLaneSpeeds(simulatorData.egoCar, simulatorData.otherCars);
    spdlog::trace("[planPath] Lane Speeds: {}, {}, {}", laneSpeeds[0], laneSpeeds[1], laneSpeeds[2]);

    if (m_currentLaneD == D_LEFT_LANE && laneSpeeds[1] - LANE_CHANGE_COST > laneSpeeds[0])
    {
        spdlog::info("[planPath] Changing to middle lane.");
        m_currentLaneD = D_MIDDLE_LANE;
    }
    else if (m_currentLaneD == D_RIGHT_LANE && laneSpeeds[1] - LANE_CHANGE_COST > laneSpeeds[2])
    {
        spdlog::info("[planPath] Changing to middle lane.");
        m_currentLaneD = D_MIDDLE_LANE;
    }
    else if (m_currentLaneD == D_MIDDLE_LANE && (laneSpeeds[0] - LANE_CHANGE_COST > laneSpeeds[1] || laneSpeeds[2] - LANE_CHANGE_COST > laneSpeeds[1]))
    {
        if (laneSpeeds[0] > laneSpeeds[2])
        {
            spdlog::info("[planPath] Changing to left lane.");
            m_currentLaneD = D_LEFT_LANE;
        }
        else
        {
            spdlog::info("[planPath] Changing to right lane.");
            m_currentLaneD = D_RIGHT_LANE;
        }
    }

    std::pair<std::vector<double>, std::vector<double>> xyTrajectory = genPath(simulatorData.egoCar, kinematics, simulatorData.otherCars);

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

    spdlog::trace("XY history queue size is now ({}, {})", m_trajectoryHistoryX.size(), m_trajectoryHistoryY.size());
}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::genPath(
    const EgoCar& egoCar, const Kinematics& xyKinematics, const std::vector<OtherCar>& otherCars)
{
    // Account for max acceleration
    spdlog::trace("[genPath] EgoCar speed: {}", MPH_TO_METRES_PER_SECOND(egoCar.speed));
    double maxSpeed = std::min(SPEED_LIMIT_METRES_PER_SECOND, MPH_TO_METRES_PER_SECOND(egoCar.speed) + MAX_ACCELERATION);

    // Estimate target S point, as well as xy speed vector
    const double nextS = egoCar.s + maxSpeed * KEEP_LANE_DURATION_SECONDS;
    const double auxS = nextS - 0.01;

    double nextX, nextY;
    std::tie(nextX, nextY) = getXY(nextS, m_currentLaneD, m_waypoints);

    double auxX, auxY;
    std::tie(auxX, auxY) = getXY(auxS, m_currentLaneD, m_waypoints);

    double velocityProportionX = (nextX - auxX) / distance(nextX, nextY, auxX, auxY);
    double velocityProportionY = (nextY - auxY) / distance(nextX, nextY, auxX, auxY);

    // Generate trajectories
    const std::array<double, 6> xParams = polynomialTrajectoryParameters(KEEP_LANE_DURATION_SECONDS,
        egoCar.x, xyKinematics.velocity0, xyKinematics.acceleration0,
        nextX, maxSpeed * velocityProportionX, 0.0);

    const std::array<double, 6> yParams = polynomialTrajectoryParameters(KEEP_LANE_DURATION_SECONDS,
        egoCar.y, xyKinematics.velocity1, xyKinematics.acceleration1,
        nextY, maxSpeed * velocityProportionY, 0.0);

    std::vector<double> xTrajectory = generateTrajectoryFromParams(KEEP_LANE_DURATION_SECONDS, NODE_TRAVERSAL_RATE_SECONDS, xParams);
    std::vector<double> yTrajectory = generateTrajectoryFromParams(KEEP_LANE_DURATION_SECONDS, NODE_TRAVERSAL_RATE_SECONDS, yParams);

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
    if (m_trajectoryHistoryX.size() < 4)
    {
        // Just started

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

std::array<double, 3> PathPlanner::getLaneSpeeds(EgoCar egoCar, const std::vector<OtherCar>& otherCars) const
{
    std::array<double, 3> speeds;
    const std::array<double, 3> lanes {D_LEFT_LANE, D_MIDDLE_LANE, D_RIGHT_LANE};

    for (auto i = 0; i < lanes.size(); ++i)
    {
        egoCar.d = lanes[i];
        int carAheadIndex = getCarAhead(egoCar, otherCars);
        spdlog::trace("[getLaneSpeeds] Car ahead: {}", carAheadIndex);

        if (carAheadIndex == -1)
        {
            speeds[i] = SPEED_LIMIT_METRES_PER_SECOND;
        }
        else if (otherCars[carAheadIndex].s - egoCar.s > LANE_SPEED_FORWARD_SCAN_RANGE)
        {
            speeds[i] = SPEED_LIMIT_METRES_PER_SECOND;
        }
        else
        {
            const auto& carAhead = otherCars[carAheadIndex];
            speeds[i] = std::sqrt(carAhead.dx * carAhead.dx + carAhead.dy * carAhead.dy);
        }
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

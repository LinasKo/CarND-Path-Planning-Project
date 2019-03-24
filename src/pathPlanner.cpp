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

#include "helpers.h"


using namespace path_planning;


static constexpr double KEEP_LANE_DURATION_SECONDS = 1.0;
static constexpr double CHANGE_LANE_DURATION_SECONDS = 5.0;
static constexpr double NODE_TRAVERSAL_RATE_SECONDS = 0.02;

static constexpr double SAFETY_DISTANCE_TO_OTHER_CAR = 5.0;
static constexpr double SPEED_LIMIT = 50.0;


PathPlanner::PathPlanner(std::vector<Waypoint> waypoints) :
    m_waypoints(waypoints)
{
}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::planPath(const SimulatorResponseData& simulatorData)
{
    static bool initialized;
    if (not initialized)
    {
        m_egoCar = simulatorData.egoCar;
        initialized = true;
        return std::make_pair(std::vector<double>(), std::vector<double>());
    }

    // Waypoint closestWaypointId = ClosestWaypoint(m_egoCar, m_waypoints);
    // assert(closestWaypointId != -1);

    return std::make_pair(std::vector<double>(), std::vector<double>());
}

// std::pair<std::vector<double>, std::vector<double>> PathPlanner::GenStraightPath(const EgoCar& egoCar, const std::vector<OtherCar>& otherCars)
// {
//     // TODO: compute velocity and acceleration of egoCar --- egoCar.v, egoCar.a

//     const auto predictedCars = predictCars(otherCars, KEEP_LANE_DURATION_SECONDS);
//     int carAheadIndex = getCarAhead(egoCar, predictedCars);

//     // TODO: here it's a bit murky when deciding when to use current and predicted car positions.

//     if (carAheadIndex == -1 or otherCars[carAheadIndex].s - egoCar.s >= SAFETY_DISTANCE_TO_OTHER_CAR)
//     {
//         // No car in range
//         auto sTrajectoryParams = polynomialTrajectoryParameters(KEEP_LANE_DURATION_SECONDS,
//             egoCar.s, egoCar.sv, egoCar.sa, egoCar.s + KEEP_LANE_DURATION_SECONDS * SPEED_LIMIT, SPEED_LIMIT, 0.0);
//         auto sTrajectory = generateTrajectoryFromParams(KEEP_LANE_DURATION_SECONDS, NODE_TRAVERSAL_RATE_SECONDS, sTrajectoryParams);

//         // No car in range
//         auto dTrajectoryParams = polynomialTrajectoryParameters(KEEP_LANE_DURATION_SECONDS,
//             egoCar.d, egoCar.dv, egoCar.da, egoCar.d, 0.0, 0.0);
//         auto dTrajectory = generateTrajectoryFromParams(KEEP_LANE_DURATION_SECONDS, NODE_TRAVERSAL_RATE_SECONDS, dTrajectoryParams);

//         std::vector<double> xTrajectory, yTrajectory;
//         xTrajectory.reserve(dTrajectory.size());
//         yTrajectory.reserve(dTrajectory.size());
//         for (int i = 0; i < dTrajectory.size(); ++i)
//         {
//             double x, y;
//             std::tie(x, y) = getXY(sTrajectory[i], dTrajectory[i], m_waypoints);
//             xTrajectory.append(x);
//             yTrajectory.append(y);
//         }

//         return std:make_pair(xTrajectory, yTrajectory);

//     }
//     else
//     {
//         // Car ahead
//         const OtherCar& carAhead = otherCars[carAheadIndex];
//     }
// }

/*
 * Returns the index of vehicle in front and -1 otherwise.
 */
int PathPlanner::getCarAhead(const EgoCar& egoCar, const std::vector<OtherCar>& otherCars)
{
    int carAheadIndex = -1;
    double minSDist = std::numeric_limits<double>::max();

    for (int i = 0; i < otherCars.size(); ++i)
    {
        const OtherCar& otherCar = otherCars[i];
        if (otherCar.d == egoCar.d and otherCar.s >= egoCar.s)
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

std::vector<double> generateTrajectoryFromParams(double totalTime, double timeIncrement, const std::array<double, 6>& polyParams)
{
    std::vector<double> results;
    int resultCount = totalTime / timeIncrement;
    results.reserve(resultCount);

    for (int i = 1; i <= resultCount; ++i)
    {
        double t = timeIncrement * i;
        double val = polyParams[0] +                  polyParams[1] * t +              polyParams[2] * std::pow(t, 2) +
                     polyParams[3] * std::pow(t, 3) + polyParams[4] * std::pow(t, 4) + polyParams[5] * std::pow(t, 5);
        results.push_back(val);
    }

    return results;
}

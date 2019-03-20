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

#include "Eigen-3.3/Eigen"

#include "helpers.h"


using namespace path_planning;


static constexpr double KEEP_LANE_DURATION_SECONDS = 1.0;
static constexpr double CHANGE_LANE_DURATION_SECONDS = 5.0;


PathPlanner::PathPlanner(std::vector<Waypoint> waypoints) :
    m_waypoints(waypoints)
{
}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::planPath(SimulatorResponseData simulatorData)
{


    /* 
     * Assuming:
     * Set up already: m_waypoints, egoCar
     * Initial step accounted for already to set up egoCar in initial state
     */


    Waypoint closestWaypointId = ClosestWaypoint(m_egoCar, m_waypoints);
    assert(closestWaypointId != -1);

    return std::make_pair(std::vector<double>(), std::vector<double>());
}

/*
 * Returns the index of vehicle in front and -1 otherwise.
 */
int PathPlanner::getCarAhead(const EgoCar& egoCar, const vector<OtherCar>& otherCars)
{
    int vehicleAheadIndex = -1;
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
                vehicleAheadIndex = i;
            }
        }
    }

    return vehicleAheadIndex;
}

/*
 * Returns the index of vehicle behind and -1 otherwise.
 */
int PathPlanner::getCarBehind(const EgoCar& egoCar, const vector<OtherCar>& otherCars)
{
    int vehicleAheadIndex = -1;
    double minSDist = std::numeric_limits<double>::max();

    for (int i = 0; i < otherCars.size(); ++i)
    {
        const OtherCar& otherCar = otherCars[i]; 
        if (otherCar.d == egoCar.d and otherCar.s <= egoCar.s)
        {
            double sDist = egoCar.s - otherCar.s;
            if (sDist < minSDist)
            {
                minSDist = sDist;
                vehicleAheadIndex = i;
            }
        }
    }

    return vehicleAheadIndex;
}

/*
 * Predict the locations of other vehicles on the road after some time
 *
 * WARNING: does not account for the lane curvature at the moment.
 */
std::vector<OtherCar> PathPlanner::predictCars(const std::vector<OtherCar>& otherCars, double deltaTime)
{
    std::vector<OtherCar> predictions;
    predictions.reserve(otherCars.size());

    std::transform(otherCars.begin(), otherCars.end(), predictions.begin(), [deltaTime](OtherCar otherCar){
        otherCar.x += deltaTime * otherCar.dx;
        otherCar.y += deltaTime * otherCar.dy;
        double heading = orientation(otherCar.dy, otherCar.dx);
        double newS, newD;
        std::tie(newS, newD) = getFrenet(otherCar.x, otherCar.y, heading, m_waypoints);
    });

    return predictions;
}

/*
 * Generate parameters for a quintic polynomial equation, 
 * representing a change in a value with minimum jerk minimizing changes in a variable based on start and end conditions
 */
std::array<double, 6> PathPlanner::polynomialTrajectoryParameters(double totalTime,
    double startPos, double startSpeed, double startAcc, double endPos, double endSpeed, double endAcc)
{
    Eigen::Matrix3d params;
    params <<     pow(totalTime, 3),      pow(totalTime, 4),      pow(totalTime, 5),
              3 * pow(totalTime, 2), 4  * pow(totalTime, 3), 5  * pow(totalTime, 4),
              6 * pow(totalTime, 1), 12 * pow(totalTime, 2), 20 * pow(totalTime, 3);

    Eigen::Vector3d res;
    result << endPos - (StartPos + startSpeed * totalTime + 0.5 * startAcc * pow(totalTime, 2)),
              endSpeed - (startSpeed + startAcc * totalTime),
              endAcc - startAcc;

    Eigen::Vector3d result = params.inverse() * res;

    return { startPos, startSpeed, 0.5 * startAcc, result[0], result[1], result[2] };
}

/*
 * Fit a polynomial curve of given parameters with increments of time, generating 
 */
std::vector<double> generateTrajectoryFromParams(double totalTime, double timeIncrement, std::array<double, 6> params)
{
    std::vector<double> results;
    int resultCount = totalTime / timeIncrement;
    results.reserve(resultCount);

    for (int i = 1; i <= resultCount; ++i)
    {
        double t = timeIncrement * i;
        double val = params[0] + params[1] * t + params[2] * std::pow(t, 2) + params[3] * std::pow(t, 3) + params[4] * std::pow(t, 4) + params[5] * std::pow(t, 5);
        results.push_back(val);
    }

    return results;
}

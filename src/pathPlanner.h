#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <array>
#include <vector>

#include "commonDatatypes.h"
#include "simulatorCommunication.h"


namespace path_planning
{
    class PathPlanner
    {
    public:
        PathPlanner(std::vector<Waypoint> waypoints);
        std::pair<std::vector<double>, std::vector<double>> planPath(const SimulatorResponseData& simulatorData);

    private:
        /*
        * Returns the index of vehicle in front and -1 otherwise.
        */
        int getCarAhead(const EgoCar& egoCar, const std::vector<OtherCar>& otherCars);

        /*
        * Predict the locations of other vehicles on the road after some time
        *
        * WARNING: does not account for the lane curvature at the moment.
        */
        std::vector<OtherCar> predictCars(const std::vector<OtherCar>& otherCars, double deltaTime);

        /*
        * Generate parameters for a quintic polynomial equation,
        * representing a change in a value with minimum jerk minimizing changes in a variable based on start and end conditions
        */
        static std::array<double, 6> polynomialTrajectoryParameters(
            double totalTime, double startPos, double startSpeed, double startAcc, double endPos, double endSpeed, double endAcc);

        /*
        * Fit a polynomial curve of given parameters with increments of time, generating nodes along a path
        */
        static std::vector<double> generateTrajectoryFromParams(double totalTime, double timeIncrement, const std::array<double, 6>& polyParams);

        std::vector<Waypoint> m_waypoints;
        EgoCar m_egoCar;
    };
}

#endif

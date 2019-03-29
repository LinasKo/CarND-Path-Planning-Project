#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <array>
#include <deque>
#include <vector>

#include "commonDatatypes.h"
#include "simulatorCommunication.h"


namespace
{
    struct Kinematics
    {
        Kinematics() = default;
        Kinematics(double velocity0, double acceleration0, double velocity1, double acceleration1) :
            velocity0(velocity0),
            acceleration0(acceleration0),
            velocity1(velocity1),
            acceleration1(acceleration1)
        {};

        double velocity0 { 0.0 };
        double acceleration0 { 0.0 };
        double velocity1 { 0.0 };
        double acceleration1 { 0.0 };
    };

    static constexpr double D_LEFT_LANE = 2.0;
    static constexpr double D_MIDDLE_LANE = 6.0;
    static constexpr double D_RIGHT_LANE = 10.0;
}

namespace path_planning
{
    class PathPlanner
    {
    public:
        PathPlanner(std::vector<Waypoint> waypoints);
        std::pair<std::vector<double>, std::vector<double>> planPath(const SimulatorResponseData& simulatorData);

    private:
        /*
         * Update the history of the trajectory
         */
        void updateTrajectoryHistory(const SimulatorResponseData& simulatorData);

        /*
         * Generate a XY trajectory for a straight path
         */
        std::pair<std::vector<double>, std::vector<double>> genStraightPath(
            const EgoCar& egoCar, const Kinematics& xyKinematics, const std::vector<OtherCar>& otherCars);


        /*
         * Compute S and D velocity and acceleration of the vehicle
         */
        Kinematics computeSdKinematics();

        /*
         * Compute X and Y velocity and acceleration of the vehicle
         */
        Kinematics computeXyKinematics();

        /*
        * Returns the index of vehicle in front and -1 otherwise.
        */
        int getCarAhead(const EgoCar& egoCar, const std::vector<OtherCar>& otherCars) const;

        /*
         * Get estimated speeds in each lane.
         */
        std::array<double, 3> getLaneSpeeds(EgoCar egoCar, const std::vector<OtherCar>& otherCars) const;

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

        const std::vector<Waypoint> m_waypoints;
        std::vector<double> m_prevSentTrajectoryX, m_prevSentTrajectoryY;
        std::deque<double> m_trajectoryHistoryX, m_trajectoryHistoryY;
        double m_currentLaneD { D_MIDDLE_LANE };
    };
}

#endif

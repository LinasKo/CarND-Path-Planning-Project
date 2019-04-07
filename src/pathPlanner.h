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
         * Generate a XY trajectory for a path, covering both straight-line movements, and lane changing activities
         */
        std::pair<std::vector<double>, std::vector<double>> genPath(
            const EgoCar& egoCar, const Kinematics& xyKinematics, const double maxLaneSpeed, const std::vector<OtherCar>& otherCars);

        /*
         * Generate a XY trajectory for a path, covering both straight-line movements, and lane changing activities
         */
        std::pair<std::vector<double>, std::vector<double>> genPathWithPast(
            const EgoCar& egoCar, const Kinematics& xyKinematics, const std::vector<double>& prevPathX, const std::vector<double>& prevPathY,
            const double maxLaneSpeed, const std::vector<OtherCar>& otherCars, const unsigned pastElementCount);

        /*
         * Generate XY trajectory as a spline
         */
        std::pair<std::vector<double>, std::vector<double>> genPathSpline(
            const EgoCar& egoCars, const double maxLaneSpeed, const std::vector<double>& prevPathX, const std::vector<double>& prevPathY, const unsigned keepPrevious);

        /*
         * Compute S and D velocity and acceleration of the vehicle
         */
        Kinematics computeSdKinematics();

        /*
         * Compute X and Y velocity and acceleration of the vehicle.
         */
        Kinematics computeXyKinematics();

        /*
         * Compute X and Y velocity and acceleration of the vehicle, based on the next 3 unexecuted path steps, from path, returned by the simulator.
         */
        Kinematics computeXyKinematicsHist(std::vector<double> prevPathX, std::vector<double> prevPathY, const unsigned pastElementCount);

        /*
        * Returns the index of vehicle in front and -1 otherwise.
        */
        int getCarAhead(const EgoCar& egoCar, const std::vector<OtherCar>& otherCars) const;

        /*
        * Returns the index of vehicle behind, and -1 otherwise.
        */
        int getCarBehind(const EgoCar& egoCar, const std::vector<OtherCar>& otherCars) const;

        /*
         * Checks if the given lane has a vehicle that is blocking lane changes.
         */
        bool isLaneBlocked(const int targetLaneIndex, const EgoCar& egoCar, const std::vector<OtherCar>& otherCars) const;

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

        /*
        * Convert XY pose in map coordinates to car coordinates
        */
        static std::pair<double, double> worldCoordToCarCoord(const EgoCar& egoCar, double xWorld, double yWorld);

        /*
        * Convert XY poses in map coordinates to car coordinates
        */
        static std::pair<std::vector<double>, std::vector<double>> worldCoordToCarCoord(const EgoCar& egoCar, std::vector<double> xsWorld, std::vector<double> ysWorld);

        /*
        * Convert XY pose in car coordinates to map coordinates
        */
        static std::pair<double, double> carCoordToWorldCoord(const EgoCar& egoCar, double xCar, double yCar);

        /*
        * Convert XY pose in car coordinates to map coordinates
        */
        static std::pair<std::vector<double>, std::vector<double>> carCoordToWorldCoord(const EgoCar& egoCar, std::vector<double> xsCar, std::vector<double> ysCar);


        const std::vector<Waypoint> m_waypoints;
        std::vector<double> m_prevSentTrajectoryX, m_prevSentTrajectoryY;
        std::deque<double> m_trajectoryHistoryX, m_trajectoryHistoryY;
        double m_targetLaneD { D_MIDDLE_LANE };
        int m_targetLaneIndex { 1 };  // TODO: should be merged with m_targetLaneIndex
        unsigned int m_laneChangeDelay { 5u };  // Prevent changing lanes for this many time steps
    };
}

#endif

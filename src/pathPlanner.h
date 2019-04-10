#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <array>
#include <deque>
#include <vector>

#include "spline/spline.h"

#include "commonDatatypes.h"
#include "simulatorCommunication.h"



namespace
{
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
        void updateHistory(const SimulatorResponseData& simulatorData);

        /*
         * Generate XY trajectory as a spline
         */
        std::pair<std::vector<double>, std::vector<double>> genPathSpline(
            const EgoCar& egoCars, const double maxLaneSpeed, const std::vector<double>& prevPathX, const std::vector<double>& prevPathY);

        /*
         * Generate an xy path from the spline
         */
        std::pair<std::vector<double>, std::vector<double>> splineToPath(
            const tk::spline& spl, const EgoCar& egoCar, const double maxLaneSpeed, const size_t numCommandsExecuted);

        /*
         * Check if a lane change needs to be perfromed and change target lane if so
         */
        void scheduleLaneChange(const EgoCar& egoCar, const std::array<double, 3>& laneSpeeds, const std::vector<OtherCar>& otherCars);

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
        bool isLaneBlocked(const double targetLaneD, const EgoCar& egoCar, const std::vector<OtherCar>& otherCars) const;

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
        std::vector<double> m_prevSentX, m_prevSentY;
        double m_targetLaneD { D_MIDDLE_LANE };
        int m_targetLaneIndex { 1 };  // TODO: should be merged with m_targetLaneIndex, if I ever come back to the project
        unsigned int m_laneChangeDelay { 5u };  // Prevent changing lanes initially this many timesteps

        // History
        std::deque<double> m_historyEgoX, m_historyEgoY;

        // Holds previous speeds, including previous starting speed
        std::vector<double> m_prevSegmentSpeeds {{ 0.0 }};
    };
}

#endif

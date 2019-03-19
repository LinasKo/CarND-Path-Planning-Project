#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

#include "commonDatatypes.h"
#include "simulatorCommunication.h"


namespace path_planning
{
    class PathPlanner
    {
    public:
        PathPlanner(std::vector<Waypoint> waypoints);
        std::pair<std::vector<double>, std::vector<double>> planPath(SimulatorResponseData simulatorData);

    private:
        std::vector<Waypoint> m_waypoints;
    };
}

#endif

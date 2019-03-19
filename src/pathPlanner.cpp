#include "pathPlanner.h"

#include <iostream>
#include <utility>
#include <vector>

using namespace path_planning;

// TODO: finite state machine
// TODO:

PathPlanner::PathPlanner(std::vector<Waypoint> waypoints) :
    m_waypoints(waypoints)
{
}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::planPath(SimulatorResponseData simulatorData)
{
    /**
     * TODO: define a path made up of (x,y) points that the car will visit
     *   sequentially every .02 seconds
     */
    return std::make_pair(std::vector<double>(), std::vector<double>());
}

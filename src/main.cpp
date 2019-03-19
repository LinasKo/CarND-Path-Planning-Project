#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include <uWS/uWS.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "helpers.h"

#include "pathPlanner.h"
#include "simulatorCommunication.h"

using namespace path_planning;
using namespace std::placeholders;


static const std::string WAYPOINT_MAP_FILE = "data/highway_map.csv";
static constexpr int WEBSOCKET_PORT = 4567;

/** The max s value before wrapping around the track back to 0 */
static constexpr double MAX_S = 6945.554;


int main()
{
    SimulatorCommunication simComm(WEBSOCKET_PORT);

    std::vector<Waypoint> waypoints = SimulatorCommunication::readWaypoints(WAYPOINT_MAP_FILE);
    PathPlanner pathPlanner(waypoints);
    simComm.addDataHandler(std::bind(&PathPlanner::planPath, &pathPlanner, _1));

    simComm.run();
}

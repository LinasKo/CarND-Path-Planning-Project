#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include <functional>

#include "pathPlanner.h"
#include "simulatorCommunication.h"

using namespace path_planning;
using namespace std::placeholders;

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


static const string WAYPOINT_MAP_FILE = "../data/highway_map.csv";
static constexpr int WEBSOCKET_PORT = 4567;

/** The max s value before wrapping around the track back to 0 */
static constexpr double MAX_S = 6945.554;



int main()
{
    SimulatorCommunication simComm(WEBSOCKET_PORT);

    vector<double> mapWaypointsY;
    vector<double> mapWaypointsS;
    vector<double> mapWaypointsX;
    vector<double> mapWaypointsDx;
    vector<double> mapWaypointsDy;
    SimulatorCommunication::readWaypoints(WAYPOINT_MAP_FILE, mapWaypointsX, mapWaypointsY, mapWaypointsS, mapWaypointsDx, mapWaypointsDy);

    PathPlanner pathPlanner;

    simComm.addDataHandler(std::bind(&PathPlanner::planPath, pathPlanner, _1, _2, _3, _4));

    simComm.run();
}

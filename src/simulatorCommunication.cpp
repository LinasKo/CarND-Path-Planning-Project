#include "simulatorCommunication.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "commonDatatypes.h"

#include "json.hpp"
#include <uWS/uWS.h>

using namespace path_planning;
using std::string;
using std::vector;


SimulatorCommunication::SimulatorCommunication(int websocket_port) :
    m_port { websocket_port }
{
    m_hub.onConnection([](uWS::WebSocket<uWS::SERVER> webSocket, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    m_hub.onDisconnection([](uWS::WebSocket<uWS::SERVER> webSocket, int code, char *message, size_t length) {
        webSocket.close();
        std::cout << "Disconnected" << std::endl;
    });
}

void SimulatorCommunication::readWaypoints(string mapFile, vector<double>& mapWaypointsX, vector<double>& mapWaypointsY, vector<double>& mapWaypointsS,
    vector<double>& mapWaypointsDx, vector<double>& mapWaypointsDy)
{
    std::ifstream inStreamMap(mapFile.c_str(), std::ifstream::in);

    string line;
    while (getline(inStreamMap, line))
    {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float dX;
        float dY;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> dX;
        iss >> dY;
        mapWaypointsX.push_back(x);
        mapWaypointsY.push_back(y);
        mapWaypointsS.push_back(s);
        mapWaypointsDx.push_back(dX);
        mapWaypointsDy.push_back(dY);
    }
}

void SimulatorCommunication::addDataHandler(std::function<std::pair<CoordinateXY, CoordinateXY>(CoordinateXY, CoordinateFrenet, double, double)> handler)
{
    m_hub.onMessage([&](uWS::WebSocket<uWS::SERVER> webSocket, char *data, size_t length, uWS::OpCode opCode)
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            string jsonStr = getData(data);

            if (jsonStr == "")
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                webSocket.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
            else
            {
                auto j = nlohmann::json::parse(jsonStr);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double carX = j[1]["x"];
                    double carY = j[1]["y"];
                    double carS = j[1]["s"];
                    double carD = j[1]["d"];
                    double carYaw = j[1]["yaw"];
                    double carSpeed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto prevPathX = j[1]["previous_path_x"];
                    auto prevPathY = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double endPathS = j[1]["end_path_s"];
                    double endPathD = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road.
                    auto sensorFusion = j[1]["sensor_fusion"];

                    nlohmann::json msgJson;

                    /**
                     * TODO: define a path made up of (x,y) points that the car will visit
                     *   sequentially every .02 seconds
                     */
                    CoordinateXY nextXVals;
                    CoordinateXY nextYVals;
                    // TODO: prev paths not accounted for
                    std::tie(nextXVals, nextYVals) = handler({ carX, carY }, { carS, carD }, carYaw, carSpeed);

                    msgJson["next_x"] = nextXVals;
                    msgJson["next_y"] = nextYVals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    webSocket.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
        }
    });
}

int SimulatorCommunication::run()
{
    if (m_hub.listen(m_port))
    {
        std::cout << "Listening to port " << m_port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    m_hub.run();
    return 0;
}


string SimulatorCommunication::getData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

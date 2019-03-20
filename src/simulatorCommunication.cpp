#include "simulatorCommunication.h"

#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <uWS/uWS.h>

#include "commonDatatypes.h"
#include "json/json.hpp"

using namespace path_planning;


static constexpr char WEBSOCKET_FLAG_MESSAGE = '4';
static constexpr char WEBSOCKET_FLAG_EVENT = '2';

static const std::string SIM_MESSAGE_MANUAL_DRIVING = "42[\"manual\",{}]";


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

std::vector<Waypoint> SimulatorCommunication::readWaypoints(std::string mapFile)
{
    std::vector<Waypoint> waypoints;

    std::ifstream inStreamMap(mapFile.c_str(), std::ifstream::in);
    assert(inStreamMap);

    std::string line;
    while (getline(inStreamMap, line))
    {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float normX;
        float normY;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> normX;
        iss >> normY;
        waypoints.push_back({x, y, s, normX, normY});
    }
    return waypoints;
}

void SimulatorCommunication::addDataHandler(std::function<std::pair<std::vector<double>, std::vector<double>>(const SimulatorResponseData&)> handler)
{
    m_hub.onMessage([handler](uWS::WebSocket<uWS::SERVER> webSocket, char *data, size_t length, uWS::OpCode opCode)
    {
        /* "42" at the start of the message means there's a websocket message event.
         * The 4 signifies a websocket message
         * The 2 signifies a websocket event */
        if (length && length > 2 && data[0] == WEBSOCKET_FLAG_MESSAGE && data[1] == WEBSOCKET_FLAG_EVENT)
        {
            std::string jsonStr = getData(data);

            if (jsonStr == "")
            {
                // Manual driving
                webSocket.send(SIM_MESSAGE_MANUAL_DRIVING.data(), SIM_MESSAGE_MANUAL_DRIVING.length(), uWS::OpCode::TEXT);
                return;
            }

            SimulatorResponseData simulatorResponse = parseData(jsonStr);

            std::vector<double> nextXVals, nextYVals;
            std::tie(nextXVals, nextYVals) = handler(simulatorResponse);

            nlohmann::json msgJson;
            msgJson["next_x"] = nextXVals;
            msgJson["next_y"] = nextYVals;

            auto msg = "42[\"control\"," + msgJson.dump() + "]";
            webSocket.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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


std::string SimulatorCommunication::getData(std::string messageString)
{
    auto found_null = messageString.find("null");
    auto b1 = messageString.find_first_of("[");
    auto b2 = messageString.find_first_of("}");
    if (found_null != std::string::npos)
    {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos)
    {
        return messageString.substr(b1, b2 - b1 + 2);
    }
    return "";
}

SimulatorResponseData SimulatorCommunication::parseData(std::string jsonString)
{
    assert(jsonString != "");

    auto jsonObj = nlohmann::json::parse(jsonString);
    std::string event = jsonObj[0].get<std::string>();

    if (event == "telemetry")
    {
        auto messageData = jsonObj[1];

        EgoCarData egoCar {
            .x = messageData["x"],
            .y = messageData["y"],
            .s = messageData["s"],
            .d = messageData["d"],
            .yaw = messageData["yaw"],
            .speed = messageData["speed"]
        };

        // Previous path data given to the Planner
        std::vector<double> prevPathX = messageData["previous_path_x"];
        std::vector<double> prevPathY = messageData["previous_path_y"];
        // Previous path's end s and d values
        double endPathS = messageData["end_path_s"];
        double endPathD = messageData["end_path_d"];

        /** Sensor Fusion Data, a list of all other cars on the same side of the road. */
        std::vector<OtherCar> otherCars;
        for (const std::vector<double>& otherCarVector : messageData["sensor_fusion"])
        {
            otherCars.push_back({
                .id = otherCarVector[0],
                .x = otherCarVector[1],
                .y = otherCarVector[2],
                .s = otherCarVector[3],
                .d = otherCarVector[4],
                .dx = otherCarVector[5],
                .dy = otherCarVector[6]
            });
        }

        return { egoCar, prevPathX, prevPathY, endPathS, endPathD, otherCars };
    }
    else
    {
        std::cerr << "Invalid event received: '" << event << "'" << std::endl;
        exit(1);
    }

}

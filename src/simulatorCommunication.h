#ifndef SIMULATOR_COMMUNICATION_H
#define SIMULATOR_COMMUNICATION_H

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "commonDatatypes.h"

#include <uWS/uWS.h>


namespace path_planning
{
    class SimulatorCommunication
    {
    public:
        explicit SimulatorCommunication(int websocketPort);

        /**
         * Read map values for waypoint's x,y,s and normalized normal vectors
         */
        static std::vector<Waypoint> readWaypoints(const std::string& mapFile);

        void addDataHandler(std::function<std::pair<std::vector<double>, std::vector<double>>(const SimulatorResponseData&)> handler);

        int run();

    private:
        /**
         * Checks if the SocketIO event has JSON data.
         * If there is data the JSON object in string format will be returned,
         * else the empty string "" will be returned.
         */
        static std::string getData(const std::string& messageString);
        static SimulatorResponseData parseData(const std::string& jsonString);

        const int m_port { 0 };
        uWS::Hub m_hub {};
    };
}

#endif

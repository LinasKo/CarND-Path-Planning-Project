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
        explicit SimulatorCommunication(int websocket_port);

        /**
         * Load up map values for waypoint's x,y,s and d normalized normal vectors
         */
        static void readWaypoints(std::string mapFile, std::vector<double>& mapWaypointsX,
            std::vector<double>& mapWaypointsY, std::vector<double>& mapWaypointsS, std::vector<double>& mapWaypointsDx,
            std::vector<double>& mapWaypointsDy);

        void addDataHandler(std::function<std::pair<CoordinateXY, CoordinateXY>(CoordinateXY, CoordinateFrenet, double, double)> handler);

        int run();

    private:
        /**
         * Checks if the SocketIO event has JSON data.
         * If there is data the JSON object in string format will be returned,
         * else the empty string "" will be returned.
         */
        static std::string getData(std::string s);

        const int m_port { 0 };
        uWS::Hub m_hub {};
    };
}

#endif

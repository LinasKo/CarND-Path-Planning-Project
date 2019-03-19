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
    struct SimulatorResponseData
    {
        EgoCarData egoCarData;

        // Previous path data given to the Planner
        std::vector<double> prevPathX;
        std::vector<double> prevPathY;

        // Previous path's end s and d values
        double endPathS;
        double endPathD;

        std::vector<OtherCarData> otherCars;
    };

    class SimulatorCommunication
    {
    public:
        explicit SimulatorCommunication(int websocket_port);

        /**
         * Read map values for waypoint's x,y,s and normalized normal vectors
         */
        static std::vector<Waypoint> readWaypoints(std::string mapFile);

        void addDataHandler(std::function<std::pair<std::vector<double>, std::vector<double>>(const SimulatorResponseData&)> handler);

        int run();

    private:
        /**
         * Checks if the SocketIO event has JSON data.
         * If there is data the JSON object in string format will be returned,
         * else the empty string "" will be returned.
         */
        static std::string getData(std::string messageString);
        static SimulatorResponseData parseData(std::string jsonString);

        const int m_port { 0 };
        uWS::Hub m_hub {};
    };
}

#endif

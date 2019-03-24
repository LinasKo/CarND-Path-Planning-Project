#ifndef COMMON_DATATYPES_H
#define COMMON_DATATYPES_H


namespace path_planning
{
    struct EgoCar
    {
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
    };

    struct OtherCar
    {
        double id;
        double x;
        double y;
        double s;
        double d;
        double dx;
        double dy;
    };

    struct Waypoint
    {
        double x;
        double y;
        float s;
        float normX;
        float normY;
    };

    struct SimulatorResponseData
    {
        EgoCar egoCar;

        // Previous path data given to the Planner
        std::vector<double> prevPathX;
        std::vector<double> prevPathY;

        // Previous path's end s and d values
        double endPathS;
        double endPathD;

        std::vector<OtherCar> otherCars;
    };
}

#endif

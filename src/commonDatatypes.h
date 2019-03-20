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
}

#endif

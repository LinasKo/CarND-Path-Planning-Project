#ifndef COMMON_DATATYPES_H
#define COMMON_DATATYPES_H

#include <utility>

namespace path_planning
{
    struct EgoCarData
    {
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
    };

    struct OtherCarData
    {
        double id;
        double x;
        double y;
        double s;
        double d;
        double dx;
        double dy;
    };
}

#endif

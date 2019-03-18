#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "commonDatatypes.h"


namespace path_planning
{
    class PathPlanner
    {
    public:
        std::pair<CoordinateXY, CoordinateXY> planPath(CoordinateXY carXY, CoordinateFrenet carFrenet, double carYaw, double carSpeed);
    };
}

#endif

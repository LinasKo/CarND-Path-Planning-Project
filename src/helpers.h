#ifndef HELPERS_H
#define HELPERS_H

#include <cassert>
#include <limits>
#include <math.h>
#include <utility>
#include <vector>

#include "commonDatatypes.h"


namespace path_planning
{
    constexpr double MAP_CENTER_X = 1000;
    constexpr double MAP_CENTER_Y = 2000;

    // angle-related functions
    constexpr double pi() { return M_PI; }
    double deg2rad(double x) { return x * pi() / 180; }
    double rad2deg(double x) { return x * 180 / pi(); }
    double orientation(double y, double x) { return atan2(y, x); }

    double distance(double x1, double y1, double x2, double y2)
    {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    /*
     * Returns the closest waypoint, even if it behind the given point. Returns -1
     */
    int closestWaypoint(double x, double y, const std::vector<Waypoint>& waypoints)
    {
        assert(waypoints.size() > 0);

        double closestLen = std::numeric_limits<double>::max();
        int closestWaypointIndex = -1;

        for (int i = 0; i < waypoints.size(); ++i)
        {
            const Waypoint& wp = waypoints[i];
            const double dist = distance(x, y, wp.x, wp.y);
            if (dist < closestLen)
            {
                closestLen = dist;
                closestWaypointIndex = i;
            }
        }

        return closestWaypointIndex;
    }

    /*
     * Returns the closest waypoint ahead of the vehicle / point by making sure to take the orientation into account.
     */
    int NextWaypoint(double x, double y, double theta, const std::vector<Waypoint>& waypoints)
    {
        int closestWaypointIndex = closestWaypoint(x, y, waypoints);
        Waypoint closestWaypoint = waypoints[closestWaypointIndex];

        double heading = atan2((closestWaypoint.y - y), (closestWaypoint.x - x));
        double angle = fabs(theta - heading);
        angle = std::min(2 * pi() - angle, angle);

        if (angle > pi() / 2)
        {
            // Assume waypoints are ordered
            ++closestWaypointIndex;

            // Wrap around to waypoint 0
            if (closestWaypointIndex == waypoints.size())
            {
                closestWaypointIndex = 0;
            }
        }

        return closestWaypointIndex;
    }

    /*
     * Transform from Cartesian x,y coordinates to Frenet s,d coordinates
     */
    std::pair<double, double> getFrenet(double x, double y, double theta, const std::vector<Waypoint>& waypoints)
    {
        int nextWpIndex = NextWaypoint(x, y, theta, waypoints);
        const Waypoint& nextWp = waypoints[nextWpIndex];

        int prevWpIndex = nextWpIndex - 1;
        if (nextWpIndex == 0)
        {
            prevWpIndex = waypoints.size() - 1;
        }
        const Waypoint& prevWp = waypoints[prevWpIndex];

        double nX = nextWp.x - prevWp.x;
        double nY = nextWp.y - prevWp.y;
        double xX = x - prevWp.x;
        double yY = y - prevWp.y;

        // find the projection of x onto n
        double proj_norm = (xX * nX + yY * nY) / (nX * nX + nY * nY);
        double projX = proj_norm * nX;
        double projY = proj_norm * nY;

        double frenetD = distance(xX, yY, projX, projY);

        //see if d value is positive or negative by comparing it to a center point
        double centerX = MAP_CENTER_X - prevWp.x;
        double centerY = MAP_CENTER_Y - prevWp.y;
        double centerToPos = distance(centerX, centerY, xX, yY);
        double centerToRef = distance(centerX, centerY, projX, projY);

        if (centerToPos <= centerToRef)
        {
            frenetD *= -1;
        }

        // calculate s value
        double frenetS = 0;
        for (int i = 0; i < prevWpIndex; ++i)
        {
            frenetS += distance(waypoints[i].x, waypoints[i].y, waypoints[i + 1].x, waypoints[i + 1].y);
        }

        frenetS += distance(0, 0, projX, projY);

        return std::make_pair(frenetS, frenetD);
    }

    /*
     * Transform from Frenet s,d coordinates to Cartesian x,y
     */
    std::pair<double, double> getXY(double s, double d, const std::vector<Waypoint>& waypoints)
    {
        int prevWpIndex = -1;
        while (s > waypoints[prevWpIndex + 1].s && (prevWpIndex < (int)(waypoints.size() - 1)))
        {
            ++prevWpIndex;
        }
        assert(prevWpIndex != -1);

        const Waypoint& prevWp = waypoints[prevWpIndex];
        const Waypoint& nextWp = waypoints[(prevWpIndex + 1) % waypoints.size()];

        double heading = atan2(nextWp.y - prevWp.y, nextWp.x - prevWp.x);

        // the x,y,s along the segment
        double segS = (s - prevWp.s);
        double segX = prevWp.x + segS * cos(heading);
        double segY = prevWp.y + segS * sin(heading);

        double perpHeading = heading - pi() / 2;

        double x = segX + d * cos(perpHeading);
        double y = segY + d * sin(perpHeading);

        return std::make_pair(x, y);
    }
}

#endif

//
// Created by chmst on 10/11/2016.
//

#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <math.h>
#include <vector>
#include "WayPoint.hpp"
#include "SemanticTypes.h"


class Map {
public:
    std::vector<WayPoint> wayPoints;
    Distance max_s;

    Map() : wayPoints(), max_s(0) {}
    Map(std::vector<WayPoint> wayPoints, Distance max_s) : wayPoints(wayPoints), max_s(max_s) {}

    int ClosestWayPointIndex(const GlobalCartesianCoordinate pos) const;
    int NextWayPointIndex(const GlobalCartesianPosition pos) const;

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    FrenetCoordinate ConvertToFrenet(const GlobalCartesianPosition pos) const;

    // Transform from Frenet s,d coordinates to Cartesian x,y
    GlobalCartesianCoordinate ConvertToCartesian(const FrenetCoordinate pos);
};

#endif //MAP_H

//
// Created by chmst on 10/11/2016.
//

#ifndef WAY_POINT_HPP
#define WAY_POINT_HPP

#include <iostream>
#include <math.h>
#include "SemanticTypes.h"


class WayPoint {
public:
    const GlobalCartesianCoordinate cartesian;
    const FrenetCoordinate frenet;

    WayPoint(GlobalCartesianCoordinate cartesian, FrenetCoordinate frenet) : cartesian(cartesian), frenet(frenet) {}
};

#endif //WAY_POINT_HPP

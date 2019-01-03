#ifndef WAY_POINT_HPP
#define WAY_POINT_HPP

#include <iostream>
#include <cmath>
#include "SemanticTypes.h"


class WayPoint {
public:
    const GlobalCartesianCoordinate cartesian;
    const FrenetCoordinate frenet;
    const LocalCartesianCoordinate normalized_normal_vector;

    WayPoint(GlobalCartesianCoordinate cartesian, FrenetCoordinate frenet, LocalCartesianCoordinate normalized_normal_vector) : cartesian(cartesian), frenet(frenet), normalized_normal_vector(normalized_normal_vector) {}
};

#endif //WAY_POINT_HPP

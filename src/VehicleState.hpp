#ifndef VEHICLE_STATE_HPP
#define VEHICLE_STATE_HPP

#include <iostream>
#include <cmath>
#include "SemanticTypes.h"


class VehicleState {
public:
    int id;
    GlobalCartesianPosition cartesian;
    FrenetCoordinate frenet;
    Speed speed_x;
    Speed speed_y;
    Speed speed;

    VehicleState(int id, const GlobalCartesianPosition cartesian, const FrenetCoordinate frenet, const Speed speed)
        : id(id),
        cartesian(cartesian),
        frenet(frenet),
        speed_x(speed * cos(cartesian.theta)),
        speed_y(speed * sin(cartesian.theta)),
        speed(speed) {}

    VehicleState(int id, const GlobalCartesianCoordinate cartesian, const FrenetCoordinate frenet, const Speed speed_x, const Speed speed_y)
        : id(id),
        cartesian(cartesian.x, cartesian.y, atan2(speed_y, speed_x)),
        frenet(frenet),
        speed_x(speed_x),
        speed_y(speed_y),
        speed(sqrt(pow<2>(speed_x) + pow<2>(speed_y))) {}
};

#endif //VEHICLE_STATE_HPP

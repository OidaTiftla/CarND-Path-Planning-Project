//
// Created by chmst on 10/11/2016.
//

#ifndef VEHICLE_STATE_HPP
#define VEHICLE_STATE_HPP

#include <iostream>
#include <cmath>
#include "SemanticTypes.h"


class VehicleState {
public:
    const int id;
    const GlobalCartesianPosition cartesian;
    const FrenetCoordinate frenet;
    const Speed speed;

    VehicleState(int id, GlobalCartesianPosition cartesian, FrenetCoordinate frenet, Speed speed) : id(id), cartesian(cartesian), frenet(frenet), speed(speed) {}
};

#endif //VEHICLE_STATE_HPP

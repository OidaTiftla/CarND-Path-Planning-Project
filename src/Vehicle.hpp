//
// Created by chmst on 10/11/2016.
//

#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <iostream>
#include <math.h>
#include "SemanticTypes.h"


class Vehicle {
public:
    const GlobalCartesianPosition cartesian;
    const FrenetCoordinate frenet;
    const Speed speed;

    Vehicle(GlobalCartesianPosition cartesian, FrenetCoordinate frenet, Speed speed) : cartesian(cartesian), frenet(frenet), speed(speed) {}
};

#endif //VEHICLE_HPP

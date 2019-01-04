#ifndef COORDINATE_SYSTEM_REFERENCE_HPP
#define COORDINATE_SYSTEM_REFERENCE_HPP

#include <iostream>
#include <cmath>
#include "Units.hpp"
#include "CartesianPosition.hpp"


class CoordinateSystemReference {
public:
    GlobalCartesianPosition reference;

    CoordinateSystemReference(const GlobalCartesianPosition& reference) : reference(reference) {}
    CoordinateSystemReference(const Distance x, const Distance y, const AngleRad theta) : reference(x, y, theta) {}

    // Convert map coordinates into local coordinates of the car,
    // where the car is at (x, y) in the map and also has an orientation.
    LocalCartesianCoordinate to_local(const GlobalCartesianCoordinate& g) const {
        return LocalCartesianCoordinate(
            cos(this->reference.theta) * (g.x - this->reference.coord.x) + sin(this->reference.theta) * (g.y - this->reference.coord.y),
            -sin(this->reference.theta) * (g.x - this->reference.coord.x) + cos(this->reference.theta) * (g.y - this->reference.coord.y)
            );
    }

    // Convert local coordinates of the car into map coordinates,
    // where the car is at (x, y) in the map and also has an orientation.
    GlobalCartesianCoordinate to_global(const LocalCartesianCoordinate& l) const {
        return GlobalCartesianCoordinate(
            cos(this->reference.theta) * l.x - sin(this->reference.theta) * l.y + this->reference.coord.x,
            sin(this->reference.theta) * l.x + cos(this->reference.theta) * l.y + this->reference.coord.y
            );
    }

    // Convert map coordinates into local coordinates of the car,
    // where the car is at (x, y) in the map and also has an orientation.
    LocalCartesianPosition to_local(const GlobalCartesianPosition& g) const {
        auto coordl = this->to_local(g.coord);
        return LocalCartesianPosition(coordl.x, coordl.y,
            g.theta - this->reference.theta);
    }

    // Convert local coordinates of the car into map coordinates,
    // where the car is at (x, y) in the map and also has an orientation.
    GlobalCartesianPosition to_global(const LocalCartesianPosition& l) const {
        auto coordg = this->to_global(l.coord);
        return GlobalCartesianPosition(coordg.x, coordg.y,
            l.theta + this->reference.theta);
    }
};

#endif //COORDINATE_SYSTEM_REFERENCE_HPP

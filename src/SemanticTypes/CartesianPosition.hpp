//
// Created by chmst on 10/11/2016.
//

#ifndef CARTESIAN_POSITION_HPP
#define CARTESIAN_POSITION_HPP

#include <iostream>
#include <math.h>
#include "Units.hpp"
#include "CartesianCoordinate.hpp"


template <CoordinateReference TReference>
class CartesianPosition {
public:
    const CartesianCoordinate<TReference> coord;
    const AngleRad theta;

    CartesianPosition() : coord(), theta(0) {}
    CartesianPosition(const Distance x, const Distance y, const AngleRad theta) : coord(x, y), theta(theta) {}

    Distance DistanceTo(const CartesianPosition<TReference> p) const {
        return coord.DistanceTo(p.coord);
    }

    Distance DistanceTo(const CartesianCoordinate<TReference> c) const {
        return coord.DistanceTo(c);
    }

    AngleRad AngleTo(const CartesianPosition<TReference> p) const {
        return NormalizeAroundZero(coord.AngleTo(p.coord) - theta);
    }

    AngleRad AngleTo(const CartesianCoordinate<TReference> c) const {
        return NormalizeAroundZero(coord.AngleTo(c) - theta);
    }
};

using LocalCartesianPosition = CartesianPosition<CoordinateReference::Local>;
using GlobalCartesianPosition = CartesianPosition<CoordinateReference::Global>;


std::ostream& operator<<(std::ostream& out, const LocalCartesianPosition& o) {
    return out << "{local x:" << o.coord.x << " y:" << o.coord.y << " theta:" << o.theta << "}";
}

std::ostream& operator<<(std::ostream& out, const GlobalCartesianPosition& o) {
    return out << "{global x:" << o.coord.x << " y:" << o.coord.y << " theta:" << o.theta << "}";
}

#endif //CARTESIAN_POSITION_HPP
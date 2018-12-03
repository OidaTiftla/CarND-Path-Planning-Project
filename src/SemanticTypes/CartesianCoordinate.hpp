//
// Created by chmst on 10/11/2016.
//

#ifndef CARTESIAN_COORDINATE_HPP
#define CARTESIAN_COORDINATE_HPP

#include <iostream>
#include <math.h>
#include "Units.hpp"

enum class CoordinateReference {
    Global,
    Local,
};

template <CoordinateReference TReference>
class CartesianCoordinate {
public:
    const Distance x;
    const Distance y;

    CartesianCoordinate() : x(0), y(0) {}
    CartesianCoordinate(const Distance x, const Distance y) : x(x), y(y) {}

    Distance DistanceTo(const CartesianCoordinate<TReference> c) const {
        return Distance(sqrt((x - c.x).value*(x - c.x).value + (y - c.y).value*(y - c.y).value));
    }

    AngleRad AngleTo(const CartesianCoordinate<TReference> p) const {
        return NormalizeAroundZero(AngleRad(atan2((p.y - y).value, (p.x - x).value)));
    }
};

using LocalCartesianCoordinate = CartesianCoordinate<CoordinateReference::Local>;
using GlobalCartesianCoordinate = CartesianCoordinate<CoordinateReference::Global>;


std::ostream& operator<<(std::ostream& out, const LocalCartesianCoordinate& o) {
    return out << "{local x:" << o.x << " y:" << o.y << "}";
}

std::ostream& operator<<(std::ostream& out, const GlobalCartesianCoordinate& o) {
    return out << "{global x:" << o.x << " y:" << o.y << "}";
}

GlobalCartesianCoordinate operator+(const GlobalCartesianCoordinate& a, const LocalCartesianCoordinate& b) {
    return GlobalCartesianCoordinate(a.x + b.x, a.y + b.y);
}

GlobalCartesianCoordinate operator+(const LocalCartesianCoordinate& a, const GlobalCartesianCoordinate& b) {
    return b + a;
}

GlobalCartesianCoordinate operator-(const GlobalCartesianCoordinate& a, const LocalCartesianCoordinate& b) {
    return GlobalCartesianCoordinate(a.x - b.x, a.y - b.y);
}

LocalCartesianCoordinate operator-(const GlobalCartesianCoordinate& a, const GlobalCartesianCoordinate& b) {
    return LocalCartesianCoordinate(a.x - b.x, a.y - b.y);
}

LocalCartesianCoordinate operator+(const LocalCartesianCoordinate& a, const LocalCartesianCoordinate& b) {
    return LocalCartesianCoordinate(a.x + b.x, a.y + b.y);
}

LocalCartesianCoordinate operator-(const LocalCartesianCoordinate& a, const LocalCartesianCoordinate& b) {
    return LocalCartesianCoordinate(a.x - b.x, a.y - b.y);
}

#endif //CARTESIAN_COORDINATE_HPP

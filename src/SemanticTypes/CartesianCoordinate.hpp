#ifndef CARTESIAN_COORDINATE_HPP
#define CARTESIAN_COORDINATE_HPP

#include <iostream>
#include <cmath>
#include "Units.hpp"

enum class CoordinateReference {
    Global,
    Local,
};

template <CoordinateReference TReference>
class CartesianCoordinate {
public:
    Distance x;
    Distance y;

    CartesianCoordinate() : x(0), y(0) {}
    CartesianCoordinate(const Distance x, const Distance y) : x(x), y(y) {}

    Distance distance_to(const CartesianCoordinate<TReference> c) const {
        return Distance(sqrt(pow<2>(x - c.x) + pow<2>(y - c.y)));
    }

    AngleRad angle_to(const CartesianCoordinate<TReference> p) const {
        return normalize_around_zero(atan2(p.y - y, p.x - x));
    }
};

using LocalCartesianCoordinate = CartesianCoordinate<CoordinateReference::Local>;
using GlobalCartesianCoordinate = CartesianCoordinate<CoordinateReference::Global>;


inline std::ostream& operator<<(std::ostream& out, const LocalCartesianCoordinate& o) {
    return out << "{local x:" << o.x << " y:" << o.y << "}";
}

inline std::ostream& operator<<(std::ostream& out, const GlobalCartesianCoordinate& o) {
    return out << "{global x:" << o.x << " y:" << o.y << "}";
}

inline GlobalCartesianCoordinate operator+(const GlobalCartesianCoordinate& a, const LocalCartesianCoordinate& b) {
    return GlobalCartesianCoordinate(a.x + b.x, a.y + b.y);
}

inline GlobalCartesianCoordinate operator+(const LocalCartesianCoordinate& a, const GlobalCartesianCoordinate& b) {
    return b + a;
}

inline GlobalCartesianCoordinate operator-(const GlobalCartesianCoordinate& a, const LocalCartesianCoordinate& b) {
    return GlobalCartesianCoordinate(a.x - b.x, a.y - b.y);
}

inline LocalCartesianCoordinate operator-(const GlobalCartesianCoordinate& a, const GlobalCartesianCoordinate& b) {
    return LocalCartesianCoordinate(a.x - b.x, a.y - b.y);
}

inline LocalCartesianCoordinate operator+(const LocalCartesianCoordinate& a, const LocalCartesianCoordinate& b) {
    return LocalCartesianCoordinate(a.x + b.x, a.y + b.y);
}

inline LocalCartesianCoordinate operator-(const LocalCartesianCoordinate& a, const LocalCartesianCoordinate& b) {
    return LocalCartesianCoordinate(a.x - b.x, a.y - b.y);
}

#endif //CARTESIAN_COORDINATE_HPP

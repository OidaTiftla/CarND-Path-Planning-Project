#ifndef CARTESIAN_POSITION_HPP
#define CARTESIAN_POSITION_HPP

#include <iostream>
#include <cmath>
#include "Units.hpp"
#include "CartesianCoordinate.hpp"


template <CoordinateReference TReference>
class CartesianPosition {
public:
    CartesianCoordinate<TReference> coord;
    AngleRad theta;

    CartesianPosition() : coord(), theta(0) {}
    CartesianPosition(const Distance x, const Distance y, const AngleRad theta) : coord(x, y), theta(theta) {}
    CartesianPosition(const CartesianCoordinate<TReference> coord, const AngleRad theta) : coord(coord), theta(theta) {}

    Distance distance_to(const CartesianPosition<TReference> p) const {
        return coord.distance_to(p.coord);
    }

    Distance distance_to(const CartesianCoordinate<TReference> c) const {
        return coord.distance_to(c);
    }

    AngleRad angle_to(const CartesianPosition<TReference> p) const {
        return normalize_around_zero(coord.angle_to(p.coord) - theta);
    }

    AngleRad angle_to(const CartesianCoordinate<TReference> c) const {
        return normalize_around_zero(coord.angle_to(c) - theta);
    }
};

using LocalCartesianPosition = CartesianPosition<CoordinateReference::Local>;
using GlobalCartesianPosition = CartesianPosition<CoordinateReference::Global>;


inline std::ostream& operator<<(std::ostream& out, const LocalCartesianPosition& o) {
    return out << "{local x:" << o.coord.x << " y:" << o.coord.y << " theta:" << o.theta << "}";
}

inline std::ostream& operator<<(std::ostream& out, const GlobalCartesianPosition& o) {
    return out << "{global x:" << o.coord.x << " y:" << o.coord.y << " theta:" << o.theta << "}";
}

#endif //CARTESIAN_POSITION_HPP

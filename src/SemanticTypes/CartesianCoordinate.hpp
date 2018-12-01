//
// Created by chmst on 10/11/2016.
//

#ifndef CARTESIAN_COORDINATE_HPP
#define CARTESIAN_COORDINATE_HPP

#include <iostream>
#include <math.h>

enum class CoordinateReference {
    Global,
    Local,
};

template <class TDistance, CoordinateReference TReference>
class CartesianCoordinate {
public:
    const TDistance x;
    const TDistance y;

    CartesianCoordinate() : x(0), y(0) {}
    CartesianCoordinate(TDistance x, TDistance y) : x(x), y(y) {}

    TDistance DistanceTo(CartesianCoordinate<TDistance, TReference> c) {
        return TDistance(sqrt((x - c.x).value*(x - c.x).value + (y - c.y).value*(y - c.y).value));
    }

    friend std::ostream& operator<< <> (std::ostream& out,
        CartesianCoordinate<TDistance, TReference>& o) {
        return out << "{x:" << o.x << " y:" << o.y << "}";
    }

    friend CartesianCoordinate<TDistance, TReference>&& operator+(CartesianCoordinate<TDistance, TReference>& a, CartesianCoordinate<TDistance, TReference>& b) {
        return CartesianCoordinate<TDistance, TReference>(a.x + b.x, a.y + b.y);
    }

    friend CartesianCoordinate<TDistance, TReference>&& operator-(CartesianCoordinate<TDistance, TReference>& a, CartesianCoordinate<TDistance, TReference>& b) {
        return CartesianCoordinate<TDistance, TReference>(a.x - b.x, a.y - b.y);
    }
};

template <class TDistance>
using LocalCartesianCoordinate = CartesianCoordinate<TDistance, CoordinateReference::Local>;
template <class TDistance>
using GlobalCartesianCoordinate = CartesianCoordinate<TDistance, CoordinateReference::Global>;

#endif //CARTESIAN_COORDINATE_HPP

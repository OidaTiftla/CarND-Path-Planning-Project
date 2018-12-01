//
// Created by chmst on 10/11/2016.
//

#ifndef FRENET_COORDINATE_HPP
#define FRENET_COORDINATE_HPP

#include <iostream>
// #include <math.h>


template <class TDistance>
class FrenetCoordinate {
public:
    const TDistance s;
    const TDistance d;

    FrenetCoordinate() : s(0), d(0) {}
    FrenetCoordinate(TDistance s, TDistance d) : s(s), d(d) {}

    // TDistance DistanceTo(FrenetCoordinate<TDistance> c) {
    //     return TDistance(sqrt((s - c.s).value*(s - c.s).value + (d - c.d).value*(d - c.d).value));
    // }

    friend std::ostream& operator<< <> (std::ostream& out,
        FrenetCoordinate<TDistance>& o) {
        return out << "{s:" << o.s << " d:" << o.d << "}";
    }

    friend FrenetCoordinate<TDistance>&& operator+(FrenetCoordinate<TDistance>& a, FrenetCoordinate<TDistance>& b) {
        return FrenetCoordinate<TDistance>(a.s + b.s, a.d + b.d);
    }

    friend FrenetCoordinate<TDistance>&& operator-(FrenetCoordinate<TDistance>& a, FrenetCoordinate<TDistance>& b) {
        return FrenetCoordinate<TDistance>(a.s - b.s, a.d - b.d);
    }
};

#endif //FRENET_COORDINATE_HPP

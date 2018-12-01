//
// Created by chmst on 10/11/2016.
//

#ifndef FRENET_COORDINATE_HPP
#define FRENET_COORDINATE_HPP

#include <iostream>
// #include <math.h>
#include "SiUnits.hpp"


class FrenetCoordinate {
public:
    const Distance s;
    const Distance d;

    FrenetCoordinate() : s(0), d(0) {}
    FrenetCoordinate(const Distance s, const Distance d) : s(s), d(d) {}

    // Distance DistanceTo(const FrenetCoordinate c) {
    //     return Distance(sqrt((s - c.s).value*(s - c.s).value + (d - c.d).value*(d - c.d).value));
    // }
};


std::ostream& operator<<(std::ostream& out, const FrenetCoordinate& o) {
    return out << "{s:" << o.s << " d:" << o.d << "}";
}

FrenetCoordinate operator+(const FrenetCoordinate& a, const FrenetCoordinate& b) {
    return FrenetCoordinate(a.s + b.s, a.d + b.d);
}

FrenetCoordinate operator-(const FrenetCoordinate& a, const FrenetCoordinate& b) {
    return FrenetCoordinate(a.s - b.s, a.d - b.d);
}

#endif //FRENET_COORDINATE_HPP

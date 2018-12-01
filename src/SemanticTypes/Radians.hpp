//
// Created by chmst on 10/11/2016.
//

#ifndef RADIANS_HPP
#define RADIANS_HPP

#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>

class Radians;

#include "Degrees.h"


class Radians {
public:
    const double value;

    Radians(double value) : value(value) {}

    Degrees ToRadians() {
        return Degrees(this->value * 180 / M_PI);
    }

    Radians NormalizeAroundZero() {
        auto v = this->value;
        while (v > M_PI) {
            v -= M_2_PI;
        }
        while (v < -M_PI) {
            v += M_2_PI;
        }
        return Radians(v);
    }

    friend std::ostream& operator<<(std::ostream& out, Radians& o) {
        return out << o.value << "rad";
    }

    friend Radians&& operator+(Radians& a, Radians& b) {
        return Radians(a.value + b.value);
    }

    friend Radians&& operator-(Radians& a, Radians& b) {
        return Radians(a.value - b.value);
    }
};

#endif //RADIANS_HPP

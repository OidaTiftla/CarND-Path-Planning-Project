//
// Created by chmst on 10/11/2016.
//

#ifndef DEGREES_HPP
#define DEGREES_HPP

#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>

class Degrees;

#include "Radians.h"


class Degrees {
public:
    const double value;

    Degrees(double value) : value(value) {}

    Radians ToRadians() {
        return Radians(this->value * M_PI / 180);
    }

    Degrees NormalizeAroundZero() {
        auto v = this->value;
        while (v > 180) {
            v -= 360;
        }
        while (v < -180) {
            v += 360;
        }
        return Degrees(v);
    }

    friend std::ostream& operator<<(std::ostream& out, Degrees& o) {
        return out << o.value << "°";
    }

    friend Degrees&& operator+(Degrees& a, Degrees& b) {
        return Degrees(a.value + b.value);
    }

    friend Degrees&& operator-(Degrees& a, Degrees& b) {
        return Degrees(a.value - b.value);
    }
};

#endif //DEGREES_HPP

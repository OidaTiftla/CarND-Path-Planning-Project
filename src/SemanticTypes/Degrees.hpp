//
// Created by chmst on 10/11/2016.
//

#ifndef DEGREES_HPP
#define DEGREES_HPP

#include <iostream>

class Degrees {
public:
    const double value;

    Degrees(const double value) : value(value) {}

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
};


std::ostream& operator<<(std::ostream& out, const Degrees& o) {
    return out << o.value << "°";
}

Degrees operator+(const Degrees& a, const Degrees& b) {
    return Degrees(a.value + b.value);
}

Degrees operator-(const Degrees& a, const Degrees& b) {
    return Degrees(a.value - b.value);
}

#endif //DEGREES_HPP

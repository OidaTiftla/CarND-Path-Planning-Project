//
// Created by chmst on 16/11/2016.
//

#include "Meters.h"

Meters::Meters(double value) : value(value) {}

std::ostream& operator<<(std::ostream& out, Meters& o) {
    return out << o.value << "m";
}

Meters && operator+(Meters & a, Meters & b) {
    return Meters(a.value + b.value);
}
Meters && operator-(Meters & a, Meters & b) {
    return Meters(a.value - b.value);
}

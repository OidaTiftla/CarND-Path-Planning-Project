//
// Created by chmst on 10/11/2016.
//

#include "Degrees.h"

Degrees::Degrees(double value) : value(value) {
}

Radians Degrees::ToRadians() {
    return Radians(this->value * M_PI / 180);
}

std::ostream& operator<<(std::ostream& out, Degrees& o) {
    return out << o.value << "°";
}

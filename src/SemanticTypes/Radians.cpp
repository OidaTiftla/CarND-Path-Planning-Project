//
// Created by chmst on 10/11/2016.
//

#include "Radians.h"

Radians::Radians(double value) : value(value) {
}

Degrees Radians::ToDegrees() {
    return Degrees(this->value * 180 / M_PI);
}

std::ostream& operator<<(std::ostream& out, Radians& o) {
    return out << o.value << "rad";
}

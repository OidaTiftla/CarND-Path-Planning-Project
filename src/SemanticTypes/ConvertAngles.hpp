//
// Created by chmst on 10/11/2016.
//

#ifndef CONVERT_ANGLES_HPP
#define CONVERT_ANGLES_HPP

#define _USE_MATH_DEFINES
#include <cmath>

#include "Degrees.hpp"
#include "Radians.hpp"


Radians ToRadians(const Degrees& deg) {
    return Radians(deg.value * M_PI / 180);
}

Degrees ToDegrees(const Radians& rad) {
    return Degrees(rad.value * 180 / M_PI);
}

#endif //CONVERT_ANGLES_HPP

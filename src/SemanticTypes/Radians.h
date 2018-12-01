//
// Created by chmst on 10/11/2016.
//

#ifndef RADIANS_H
#define RADIANS_H

#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>


class Radians;

#include "Degrees.h"


class Radians {
public:
    double value;

    Radians(double value);

    Degrees ToDegrees();

    friend std::ostream& operator<<(std::ostream& out, Radians& o);
};


#endif //RADIANS_H

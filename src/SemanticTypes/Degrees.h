//
// Created by chmst on 10/11/2016.
//

#ifndef DEGREES_H
#define DEGREES_H

#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>

class Degrees;

#include "Radians.h"


class Degrees {
public:
    double value;

    Radians ToRadians();

    Degrees(double value);

    friend std::ostream& operator<<(std::ostream& out, Degrees& o);
};


#endif //DEGREES_H

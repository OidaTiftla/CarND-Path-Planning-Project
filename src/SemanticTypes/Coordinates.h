//
// Created by chmst on 10/11/2016.
//

#ifndef COORDINATES_H
#define COORDINATES_H

#include "Coordinate.h"
#include "Meters.h"

class CarCoordinate : public Coordinate<Meters> {
public:
    CarCoordinate();
    CarCoordinate(const Meters &x, const Meters &y);
};

class TrackCoordinate : public Coordinate<Meters> {
public:
    TrackCoordinate();
    TrackCoordinate(const Meters &x, const Meters &y);
};


#endif //COORDINATES_H

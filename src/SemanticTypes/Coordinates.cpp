//
// Created by chmst on 16/11/2016.
//

#include "Coordinates.h"

CarCoordinate::CarCoordinate()
        : Coordinate() {}

CarCoordinate::CarCoordinate(const Meters &x, const Meters &y)
        : Coordinate(x, y) {}

TrackCoordinate::TrackCoordinate()
        : Coordinate() {}

TrackCoordinate::TrackCoordinate(const Meters &x, const Meters &y)
        : Coordinate(x, y) {}

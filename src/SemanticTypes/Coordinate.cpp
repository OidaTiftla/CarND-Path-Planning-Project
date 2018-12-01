//
// Created by chmst on 16/11/2016.
//

#include "Coordinate.h"

template <class TDistance>
Coordinate<TDistance>::Coordinate() : x(0), y(0) {}

template <class TDistance>
Coordinate<TDistance>::Coordinate(TDistance x, TDistance y) : x(x), y(y) {}

template<class TDistance>
TDistance Coordinate<TDistance>::DistanceTo(Coordinate<TDistance> c) {
    return TDistance(sqrt((x - c.x).value*(x - c.x).value + (y - c.y).value*(y - c.y).value));
}

template <class TDistance>
std::ostream& operator<<(std::ostream& out, Coordinate<TDistance>& o) {
    return out << "{x:" << o.x << " y:" << o.y << "}";
}

// Explicitly instantiate the template, and its member definitions
#include "Meters.h"

template class Coordinate<Meters>;
template std::ostream& operator<< <Meters>(std::ostream& out,
    Coordinate<Meters>& o);

//
// Created by chmst on 10/11/2016.
//

#ifndef COORDINATE_H
#define COORDINATE_H

#include <iostream>
#include <math.h>


template <class TDistance>
class Coordinate;

template <class TDistance>
std::ostream& operator<<(std::ostream& out, Coordinate<TDistance>& o);

template <class TDistance>
class Coordinate {
public:
    TDistance x;
    TDistance y;

    Coordinate();
    Coordinate(TDistance x, TDistance y);

	TDistance DistanceTo(Coordinate<TDistance> c);
    friend std::ostream& operator<< <> (std::ostream& out,
                                        Coordinate<TDistance>& o);
};


#endif //COORDINATE_H

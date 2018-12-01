//
// Created by chmst on 10/11/2016.
//

#ifndef METERS_H
#define METERS_H

#include <iostream>



class Meters {
public:
    double value;

    Meters(double value);

    friend std::ostream& operator<<(std::ostream& out, Meters& o);
    friend Meters&& operator+(Meters&a, Meters&b);
    friend Meters&& operator-(Meters&a, Meters&b);

};


#endif //METERS_H

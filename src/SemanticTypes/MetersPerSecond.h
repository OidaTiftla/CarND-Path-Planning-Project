#ifndef METERSPERSECOND_H
#define METERSPERSECOND_H

#include <iostream>



class MetersPerSecond {
public:
    double value;

    MetersPerSecond(double value);

    friend std::ostream& operator<<(std::ostream& out, MetersPerSecond& o);
};


#endif // METERSPERSECOND_H

#include "MetersPerSecond.h"

MetersPerSecond::MetersPerSecond(double value) : value(value) {
}

std::ostream& operator<<(std::ostream& out, MetersPerSecond& o) {
    return out << o.value << "m";
}

//
// Created by chmst on 10/11/2016.
//

#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <sstream>


#define LOG_LEVEL 1

static std::stringstream nullstream;

inline std::ostream& log(int level) {
    if (level > LOG_LEVEL) {
        nullstream.seekp(0);
        return nullstream;
    } else {
        return std::cout;
    }
}

#endif //LOG_H

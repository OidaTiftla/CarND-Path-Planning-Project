//
// Created by chmst on 10/11/2016.
//

#include "log.h"


static int log_level = 0;
static std::stringstream nullstream;

void set_log_level(int level) {
    log_level = level;
}

std::ostream& log(int level) {
    if (level > log_level) {
        nullstream.seekp(0);
        return nullstream;
    } else {
        return std::cout;
    }
}

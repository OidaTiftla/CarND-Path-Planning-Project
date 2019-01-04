#include "log.h"
#include <sstream>


static int log_level = 0;
static std::stringstream nullstream;

std::ostream& log(int level) {
    if (level > log_level) {
        nullstream.seekp(0);
        return nullstream;
    } else {
        return std::cout;
    }
}

LogLevelStack::LogLevelStack(int level) {
    this->old_log_level = log_level;
    log_level = level;
}

LogLevelStack::~LogLevelStack() {
    log_level = this->old_log_level;
}

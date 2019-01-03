#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <sstream>


std::ostream& log(int level);


class LogLevelStack {
    int old_log_level = 0;

public:
    LogLevelStack(int level);
    ~LogLevelStack();
};


#endif //LOG_H

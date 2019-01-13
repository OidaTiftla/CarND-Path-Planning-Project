#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <string>


std::ostream& log(int level);


class LogLevelStack {
    int old_log_level = 0;

public:
    LogLevelStack(int level);
    ~LogLevelStack();
};

void log_signal(std::string name, double value);
void log_set_time(double time_s);
void plot_signals();


#endif //LOG_H

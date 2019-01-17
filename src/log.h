#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <string>


std::ostream& log(int level);


class LogLevelStack {
    int old_log_level = 0;
    std::string old_log_prefix = "";

public:
    LogLevelStack(int level, std::string prefix = "");
    ~LogLevelStack();
};

#if PLOTSIGNALS
void log_signal(std::string name, double value);
void log_set_time(double time_s);
void plot_signals();
#endif


#endif //LOG_H

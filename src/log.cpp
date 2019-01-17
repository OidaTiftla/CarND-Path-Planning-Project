#include "log.h"
#include <sstream>
#if PLOTSIGNALS
#include <list>
#include <map>
#include "gnuplot-iostream.h"
#endif


static int log_level = 0;
static std::string log_prefix = "";
static std::stringstream nullstream;

std::ostream& log(int level) {
    if (level > log_level) {
        nullstream.seekp(0);
        return nullstream;
    } else {
        std::cout << log_prefix;
        return std::cout;
    }
}

LogLevelStack::LogLevelStack(int level, std::string prefix /*= ""*/) {
    this->old_log_level = log_level;
    log_level = level;
    this->old_log_prefix = log_prefix;
    log_prefix = prefix;
}

LogLevelStack::~LogLevelStack() {
    log_level = this->old_log_level;
    log_prefix = this->old_log_prefix;
}

#if PLOTSIGNALS
static double log_signal_time = 0;
static std::map<std::string, std::list<std::pair<double, double>>> log_signal_values;
void log_signal(std::string name, double value) {
    if (log_signal_values.find(name) == log_signal_values.end()) {
        log_signal_values[name] = std::list<std::pair<double, double>>();
    }
    log_signal_values[name].push_back(std::make_pair(log_signal_time, value));
}

void log_set_time(double time_s) {
    log_signal_time = time_s;
}

static Gnuplot log_signal_gp;
void plot_signals() {
    auto max_time_to_show = 45;
    for (auto it = log_signal_values.begin(); it != log_signal_values.end(); ++it) {
    	while (it->second.size() > 0
            && log_signal_time - it->second.front().first > max_time_to_show) {
            it->second.pop_front();
        }
    }

	log_signal_gp << "plot";
    bool first = true;
    for (auto it = log_signal_values.begin(); it != log_signal_values.end(); ++it) {
        if (!first) {
    	    log_signal_gp << ",";
        } else {
            first = false;
        }
	    log_signal_gp << " '-' with lines title '" << it->first << "'";
    }
	log_signal_gp << "\n";
    for (auto it = log_signal_values.begin(); it != log_signal_values.end(); ++it) {
    	log_signal_gp.send1d(it->second);
    }
    log_signal_gp.flush();
}
#endif

#include "log.h"
#include <sstream>
#include <vector>
#include <map>
#include "gnuplot-iostream.h"


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

static double log_signal_time = 0;
static std::map<std::string, std::vector<std::pair<double, double>>> log_signal_values;
void log_signal(std::string name, double value) {
    if (log_signal_values.find(name) == log_signal_values.end()) {
        log_signal_values[name] = std::vector<std::pair<double, double>>();
    }
    log_signal_values[name].push_back(std::make_pair(log_signal_time, value));
}

void log_set_time(double time_s) {
    log_signal_time = time_s;
}

static Gnuplot log_signal_gp;
void plot_signals() {
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
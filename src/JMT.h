#ifndef JMT_H
#define JMT_H

#include <iostream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"

#include "WayPoint.hpp"
#include "SemanticTypes.h"
#include "VehicleState.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;


struct State {
    Distance x = 0_m;
    Speed x_dot = 0_m / 1_s;
    Acceleration x_dot_dot = 0_m / 1_s / 1_s;
};

struct Coefficients {
    Distance a_0 = 0_m;
    Speed a_1 = 0_m / 1_s;
    Acceleration a_2 = 0_m / 1_s / 1_s;
    Jerk a_3 = 0_m / 1_s / 1_s / 1_s;
    Value<SiUnit<1, 0, -4, 0, 0, 0, 0>> a_4 = 0_m / 1_s / 1_s / 1_s / 1_s;
    Value<SiUnit<1, 0, -5, 0, 0, 0, 0>> a_5 = 0_m / 1_s / 1_s / 1_s / 1_s / 1_s;

    Distance eval(const Time t) {
        return a_0 + a_1 * pow<1>(t) + a_2 * pow<2>(t) + a_3 * pow<3>(t) + a_4 * pow<4>(t) + a_5 * pow<5>(t);
    }

    Speed eval_derivative(const Time t) {
        return a_1 + 2 * a_2 * pow<1>(t) + 3 * a_3 * pow<2>(t) + 4 * a_4 * pow<3>(t) + 5 * a_5 * pow<4>(t);
    }

    Acceleration eval_derivative2(const Time t) {
        return 2 * a_2 + 3 * 2 * a_3 * pow<1>(t) + 4 * 3 * a_4 * pow<2>(t) + 5 * 4 * a_5 * pow<3>(t);
    }

    Jerk eval_derivative3(const Time t) {
        return 3 * 2 * a_3 + 4 * 3 * 2 * a_4 * pow<1>(t) + 5 * 4 * 3 * a_5 * pow<2>(t);
    }

    Value<SiUnit<1, 0, -4, 0, 0, 0, 0>> eval_derivative4(const Time t) {
        return 4 * 3 * 2 * a_4 + 5 * 4 * 3 * 2 * a_5 * pow<1>(t);
    }

    Value<SiUnit<1, 0, -5, 0, 0, 0, 0>> eval_derivative5(const Time t) {
        return 5 * 4 * 3 * 2 * a_5;
    }
};

Coefficients JMT(const State start, const State end, const Time delta_time);

#endif //JMT_H
#include "JMT.h"


Coefficients JMT(const State start, const State end, const Time delta_time) {
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficient in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    Eigen::MatrixXd A(3, 3);
    A << 1 * pow<3>(delta_time).value, 1 * pow<4>(delta_time).value, 1 * pow<5>(delta_time).value,
        3 * pow<2>(delta_time).value, 4 * pow<3>(delta_time).value, 5 * pow<4>(delta_time).value,
        6 * pow<1>(delta_time).value, 12 * pow<2>(delta_time).value, 20 * pow<3>(delta_time).value;
    Eigen::VectorXd b(3);
    auto b_0 = end.x - (start.x + start.x_dot * delta_time + 0.5 * start.x_dot_dot * pow<2>(delta_time));
    auto b_1 = end.x_dot - (start.x_dot + start.x_dot_dot * delta_time);
    auto b_2 = end.x_dot_dot - start.x_dot_dot;
    b << b_0.value,
        b_1.value,
        b_2.value;

    // solve Ax = b
    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
    // or (from solution)
    //Eigen::MatrixXd Ai = A.inverse();
    //Eigen::VectorXd x = Ai * b;

    Coefficients coeff;
    coeff.a_0 = start.x;
    coeff.a_1 = start.x_dot;
    coeff.a_2 = 0.5 * start.x_dot_dot;
    coeff.a_3 = x[0] * 1_m / 1_s / 1_s / 1_s;
    coeff.a_4 = x[1] * 1_m / 1_s / 1_s / 1_s / 1_s;
    coeff.a_5 = x[2] * 1_m / 1_s / 1_s / 1_s / 1_s / 1_s;
    return coeff;
}

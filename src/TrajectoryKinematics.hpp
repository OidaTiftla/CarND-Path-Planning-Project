//
// Created by chmst on 10/11/2016.
//

#ifndef TRAJECTORY_KINEMATICS_HPP
#define TRAJECTORY_KINEMATICS_HPP

#include <cmath>
#include "SemanticTypes.h"
#include "VehicleState.hpp"


struct TrajectoryKinematics {
    VehicleState initial_state;
    int initial_lane;
    bool is_trajectory_possible;
    VehicleState target_state;
    int target_lane;
    int intended_lane;
    Acceleration constant_acceleration = 0_m / 1_s / 1_s;
    Time time_horizon = 0_s;
    int preceding_vehicle_id = -1;

    TrajectoryKinematics(
        const VehicleState initial_state,
        const int initial_lane,
        const bool is_trajectory_possible,
        const VehicleState target_state,
        const int target_lane,
        const int intended_lane,
        const Acceleration constant_acceleration,
        const Time time_horizon,
        const int preceding_vehicle_id)
        : initial_state(initial_state),
          initial_lane(initial_lane),
          is_trajectory_possible(is_trajectory_possible),
          target_state(target_state),
          target_lane(target_lane),
          intended_lane(intended_lane),
          constant_acceleration(constant_acceleration),
          time_horizon(time_horizon),
          preceding_vehicle_id(preceding_vehicle_id) {}
};

#endif //TRAJECTORY_KINEMATICS_HPP

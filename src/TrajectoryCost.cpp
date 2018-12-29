//
// Created by chmst on 10/11/2016.
//

#include "TrajectoryCost.h"


float TrajectoryCost::calculate_cost(const TrajectoryKinematics trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) {
    return 0;
}

VehicleState TrajectoryCost::evaluate_trajectory(const TrajectoryKinematics trajectory, const Time t) const {
    auto speed = trajectory.initial_state.speed + trajectory.constant_acceleration * t;
    auto s = trajectory.initial_state.frenet.s + trajectory.initial_state.speed * t + 0.5 * trajectory.constant_acceleration * pow<2>(t);

    FrenetCoordinate frenet(s, trajectory.initial_state.frenet.d + (trajectory.target_state.frenet.d - trajectory.initial_state.frenet.d) * t / trajectory.time_horizon);
    auto cartesian = this->map.ConvertToCartesianPosition(frenet);
    return VehicleState(-1, cartesian, frenet, speed);
}

//
// Created by chmst on 10/11/2016.
//

#include "TrajectoryCost.h"


// change weights for cost functions
const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);

float TrajectoryCost::calculate_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    float cost = 0.0;
    cost += REACH_GOAL * goal_distance_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    cost += EFFICIENCY * inefficiency_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    return cost;
}

VehicleState TrajectoryCost::evaluate_trajectory(const TrajectoryKinematics &trajectory, const Time t) const {
    auto speed = trajectory.initial_state.speed + trajectory.constant_acceleration * t;
    auto s = trajectory.initial_state.frenet.s + trajectory.initial_state.speed * t + 0.5 * trajectory.constant_acceleration * pow<2>(t);

    FrenetCoordinate frenet(s, trajectory.initial_state.frenet.d + (trajectory.target_state.frenet.d - trajectory.initial_state.frenet.d) * t / trajectory.time_horizon);
    auto cartesian = this->map.ConvertToCartesianPosition(frenet);
    return VehicleState(-1, cartesian, frenet, speed);
}

float tanh(float x) {
    return (exp(x) - exp(-x)) / (exp(x) + exp(-x));
}

float sigmoid(float x) {
    return 1 / (1 + exp(-x));
}

float sigmoid_m1_1(float x) {
    return -1 + 2 * sigmoid(x);
}

float TrajectoryCost::goal_distance_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    /*
    Cost decreases as we move further along the frenet s coordinate.
    */
    auto reference_distance = this->max_speed * time_horizon;
    auto distance = this->map.GetFrenetSDistanceFromTo(trajectory.target_state.frenet.s, car.frenet.s + reference_distance);
    return sigmoid_m1_1((reference_distance - distance) / reference_distance);
}

float TrajectoryCost::inefficiency_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
    You can use the lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) function to determine the speed
    for a lane. This function is very similar to what you have already implemented in the "Implement a Second Cost Function in C++" quiz.
    */
    // if no vehicle is in the proposed lane, we can travel at target speed
    Speed proposed_speed_intended = trajectory.target_state.speed;
    // check if we need to drive behind a vehicle
    auto preceding_vehicle_id = this->map.find_next_vehicle_in_lane(car.frenet.s, trajectory.intended_lane, time_horizon, sensor_fusion);
    auto preceding_vehicle_iter = std::find_if(sensor_fusion.begin(), sensor_fusion.end(), [&preceding_vehicle_id](const VehicleState &vehicle) { return preceding_vehicle_id == vehicle.id; });
    if (preceding_vehicle_iter != sensor_fusion.end()) {
        proposed_speed_intended = preceding_vehicle_iter->speed;
    }

    // if no vehicle is in the proposed lane, we can travel at target speed
    Speed proposed_speed_final = trajectory.target_state.speed;
    // check if we need to drive behind a vehicle
    preceding_vehicle_id = this->map.find_next_vehicle_in_lane(car.frenet.s, trajectory.target_lane, time_horizon, sensor_fusion);
    preceding_vehicle_iter = std::find_if(sensor_fusion.begin(), sensor_fusion.end(), [&preceding_vehicle_id](const VehicleState &vehicle) { return preceding_vehicle_id == vehicle.id; });
    if (preceding_vehicle_iter != sensor_fusion.end()) {
        proposed_speed_final = preceding_vehicle_iter->speed;
    }

    return (2.0 * this->max_speed - proposed_speed_intended - proposed_speed_final) / this->max_speed;
}

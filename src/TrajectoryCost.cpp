#include "TrajectoryCost.h"
#include <algorithm>
#include <cmath>
#include "log.h"


// TODO: tweak weights for cost functions
const float EFFICIENCY = 1;
const float COMFORT = 1.25 * EFFICIENCY;
const float LEGALITY = 10.0 * (COMFORT + EFFICIENCY);
const float SAFETY = 1.0 * (LEGALITY + COMFORT + EFFICIENCY);
const float FEASIBILITY = 1.0 * (SAFETY + LEGALITY + COMFORT + EFFICIENCY);

float TrajectoryCost::calculate_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    auto logger = LogLevelStack(1);

    log(2) << "  + + + + + + + + + +" << std::endl;
    float cost = 0.0, c = 0.0;
    c = SAFETY * this->buffer_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    log(2) << "  + buffer_cost: " << c << std::endl;
    cost += c;
    c = FEASIBILITY * this->collision_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    log(2) << "  + collision_cost: " << c << std::endl;
    cost += c;
    c = SAFETY * this->safety_zone_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    log(2) << "  + safety_zone_cost: " << c << std::endl;
    cost += c;
    c = SAFETY * this->stays_on_road_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    log(2) << "  + stays_on_road_cost: " << c << std::endl;
    cost += c;
    c = LEGALITY * this->exceeds_speed_limit_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    log(2) << "  + exceeds_speed_limit_cost: " << c << std::endl;
    cost += c;
    c = EFFICIENCY * this->efficiency_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    log(2) << "  + efficiency_cost: " << c << std::endl;
    cost += c;
    c = COMFORT * this->total_acceleration_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    log(2) << "  + total_acceleration_cost: " << c << std::endl;
    cost += c;
    c = FEASIBILITY * this->max_acceleration_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    log(2) << "  + max_acceleration_cost: " << c << std::endl;
    cost += c;
    c = COMFORT * this->total_jerk_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    log(2) << "  + total_jerk_cost: " << c << std::endl;
    cost += c;
    c = COMFORT * this->max_jerk_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    log(2) << "  + max_jerk_cost: " << c << std::endl;
    cost += c;
    c = EFFICIENCY * this->inefficiency_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    log(2) << "  + inefficiency_cost: " << c << std::endl;
    cost += c;
    // c = EFFICIENCY * this->time_diff_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    // log(2) << "  + time_diff_cost: " << c << std::endl;
    // cost += c;
    // c = LEGALITY * this->s_diff_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    // log(2) << "  + s_diff_cost: " << c << std::endl;
    // cost += c;
    // c = LEGALITY * this->d_diff_cost(trajectory, car, timestep, time_horizon, sensor_fusion);
    // log(2) << "  + d_diff_cost: " << c << std::endl;
    // cost += c;

    return cost;
}

VehicleState TrajectoryCost::evaluate_trajectory(const TrajectoryKinematics &trajectory, const Time t) const {
    auto speed = trajectory.initial_state.speed + trajectory.constant_acceleration * t;
    auto s = trajectory.initial_state.frenet.s + trajectory.initial_state.speed * t + 0.5 * trajectory.constant_acceleration * pow<2>(t);

    FrenetCoordinate frenet(s, trajectory.initial_state.frenet.d + (trajectory.target_state.frenet.d - trajectory.initial_state.frenet.d) * t / trajectory.time_horizon);
    auto cartesian = this->map.convert_to_cartesian_position(frenet);
    return VehicleState(-1, cartesian, frenet, speed);
}

float tanh(float x) {
    return (exp(x) - exp(-x)) / (exp(x) + exp(-x));
}

float sigmoid(float x) {
    /*
    A function that returns a value between 0 and 1 for x in the
    range [-infinity, infinity].
    */
    return 1 / (1 + exp(-x));
}

float logistic(float x) {
    /*
    A function that returns a value between 0 and 1 for x in the
    range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
    */
    return -1 + 2 * sigmoid(x);
}

float TrajectoryCost::buffer_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    /*
    Penalizes getting close to other vehicles.
    */
    CoordinateSystemReference local_system(car.cartesian);
    auto t_step = std::min(1_s, std::max(0.02_s, 2_m / this->max_speed));
    auto y_min_distance = 999999_m;
    auto min_distance = 999999_m;
    for (auto it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
        // check if there is no collision
        // check the time interval [0; time_horizon]
        for (auto t = 0_s; t <= time_horizon + 0.001_s; t += t_step) {
            auto other_prediction = this->map.predict_into_future(*it, t);
            auto self_prediction = this->evaluate_trajectory(trajectory, t);
            auto local_other_prediction = local_system.to_local(other_prediction.cartesian);
            auto local_self_prediction = local_system.to_local(self_prediction.cartesian);
            auto x_dist = abs(local_other_prediction.coord.x - local_self_prediction.coord.x);
            auto y_dist = abs(local_other_prediction.coord.y - local_self_prediction.coord.y);
            auto dist = self_prediction.cartesian.distance_to(other_prediction.cartesian);

            if (x_dist < 1.2 * this->vehicle_width
                && y_dist < y_min_distance) {
                y_min_distance = y_dist;
            }
            if (dist < min_distance) {
                min_distance = dist;
            }
        }
    }
    return logistic(this->vehicle_width / min_distance + 2.0 * this->vehicle_length / y_min_distance);
}

float TrajectoryCost::collision_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    /*
    Binary cost function which penalizes collisions.
    */
    CoordinateSystemReference local_system(car.cartesian);
    auto t_step = std::min(1_s, std::max(0.02_s, 2_m / this->max_speed));
    for (auto it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
        // check if there is no collision
        // check the time interval [0; time_horizon]
        for (auto t = 0_s; t <= time_horizon + 0.001_s; t += t_step) {
            auto other_prediction = this->map.predict_into_future(*it, t);
            auto self_prediction = this->evaluate_trajectory(trajectory, t);
            auto local_other_prediction = local_system.to_local(other_prediction.cartesian);
            auto local_self_prediction = local_system.to_local(self_prediction.cartesian);
            auto x_dist = abs(local_other_prediction.coord.x - local_self_prediction.coord.x);
            auto y_dist = abs(local_other_prediction.coord.y - local_self_prediction.coord.y);

            if (x_dist < 1.2 * this->vehicle_width
                && y_dist < 1.5 * this->vehicle_length) {
                return 1;
            }
        }
    }
    return 0;
}

float TrajectoryCost::safety_zone_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    /*
    Penalizes violating the safety zone for me or the vehicle behind me.
    */
    auto t_step = std::min(1_s, std::max(0.02_s, 2_m / this->max_speed));
    auto min_actual_safety_time = 999999_s;
    for (auto it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
        // check if there is no collision
        // check the time interval [0; time_horizon]
        for (auto t = 0_s; t <= time_horizon + 0.001_s; t += t_step) {
            auto other_prediction = this->map.predict_into_future(*it, t);
            auto self_prediction = this->evaluate_trajectory(trajectory, t);
            auto d_dist = abs(other_prediction.frenet.d - self_prediction.frenet.d);

            if (d_dist < 1.2 * this->vehicle_width) {
                // each vehicle has it's own safety zone in front of it
                //     +-----------+                               |
                //     |     +  >> | <~ ~ ~ ~ safety zone ~ ~ ~ ~> |
                //     +-----------+                               |

                // check other vehicle's safety zone
                auto dist_other_to_self = this->map.get_frenet_s_distance_from_to(other_prediction.frenet.s, self_prediction.frenet.s);
                auto safety_time_other = dist_other_to_self / other_prediction.speed;
                if (min_actual_safety_time > safety_time_other) {
                    min_actual_safety_time = safety_time_other;
                }

                // check my safety zone
                auto dist_self_to_other = this->map.get_frenet_s_distance_from_to(self_prediction.frenet.s, other_prediction.frenet.s);
                auto safety_time_self = dist_self_to_other / self_prediction.speed;
                if (min_actual_safety_time > safety_time_self) {
                    min_actual_safety_time = safety_time_self;
                }
            }
        }
    }
    if (this->min_safety_zone_time <= min_actual_safety_time) {
        return 0;
    } else {
        if (min_actual_safety_time < 0.0001_s) {
            min_actual_safety_time = 0.0001_s;
        }
        return logistic((this->min_safety_zone_time - min_actual_safety_time) / min_actual_safety_time);
    }
}

float TrajectoryCost::stays_on_road_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    return 0;
}

float TrajectoryCost::exceeds_speed_limit_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    return 0;
}

float TrajectoryCost::efficiency_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    /*
    Rewards high average speeds.
    */
    auto average_speed = this->map.get_frenet_s_distance_from_to(trajectory.initial_state.frenet.s, trajectory.target_state.frenet.s) / trajectory.time_horizon;
    auto reference_speed = this->max_speed;

    auto preceding_vehicle_iter = std::find_if(sensor_fusion.begin(), sensor_fusion.end(), [&trajectory](const VehicleState &vehicle) { return trajectory.preceding_vehicle_id == vehicle.id; });
    if (preceding_vehicle_iter != sensor_fusion.end()) {
        reference_speed = preceding_vehicle_iter->speed;
    }

    if (average_speed < 0.0001_m / 1_s) {
        average_speed = 0.0001_m / 1_s;
    }
    return logistic((reference_speed - average_speed) / average_speed);
}

float TrajectoryCost::total_acceleration_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    auto t_step = std::min(1_s, std::max(0.02_s, 2_m / this->max_speed));
    auto sum_accel = 0_m / 1_s / 1_s;
    auto count = 0;
    auto last_speed = car.speed;
    // check the time interval [0; time_horizon]
    for (auto t = t_step; t <= time_horizon + 0.001_s; t += t_step) {
        auto self_prediction = this->evaluate_trajectory(trajectory, t);
        auto accel = (last_speed - self_prediction.speed) / t_step;
        sum_accel += abs(accel);
        ++count;
    }
    auto average_accel = sum_accel / count;
    return logistic(average_accel / this->max_acceleration);
}

float TrajectoryCost::max_acceleration_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    if (this->max_acceleration < trajectory.constant_acceleration) {
        return 1;
    } else {
        return 0;
    }
}

float TrajectoryCost::total_jerk_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    // auto t_step = std::min(1_s, std::max(0.02_s, 2_m / this->max_speed));
    // auto sum_jerk = 0_m / 1_s / 1_s / 1_s;
    // auto count = 0;
    // auto last_acceleration = car.acceleration;
    // // check the time interval [0; time_horizon]
    // for (auto t = t_step; t <= time_horizon + 0.001_s; t += t_step) {
    //     auto self_prediction = this->evaluate_trajectory(trajectory, t);
    //     auto jerk = (last_acceleration - self_prediction.acceleration) / t_step;
    //     sum_jerk += abs(jerk);
    //     ++count;
    // }
    // auto average_jerk = sum_jerk / count;
    // return logistic(average_jerk / this->max_jerk);
    return 0;
}

float TrajectoryCost::max_jerk_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    // if (this->max_jerk < trajectory.constant_jerk) {
    //     return 1;
    // } else {
    //     return 0;
    // }
    return 0;
}

float TrajectoryCost::inefficiency_cost(const TrajectoryKinematics &trajectory, const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
    You can use the lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) function to determine the speed
    for a lane. This function is very similar to what you have already implemented in the "Implement a Second Cost Function in C++" quiz.
    */
    // if no vehicle is in the proposed lane, we can travel at target speed
    auto proposed_speed_intended = trajectory.target_state.speed;
    // check if we need to drive behind a vehicle
    auto preceding_vehicle_id = this->map.find_next_vehicle_in_lane(car.frenet.s, trajectory.intended_lane, sensor_fusion);
    auto preceding_vehicle_iter = std::find_if(sensor_fusion.begin(), sensor_fusion.end(), [&preceding_vehicle_id](const VehicleState &vehicle) { return preceding_vehicle_id == vehicle.id; });
    if (preceding_vehicle_iter != sensor_fusion.end()
        && this->map.get_frenet_s_distance_from_to(car.frenet.s, preceding_vehicle_iter->frenet.s) < this->max_speed * time_horizon) {
        proposed_speed_intended = preceding_vehicle_iter->speed;
    }

    // if no vehicle is in the proposed lane, we can travel at target speed
    auto proposed_speed_final = trajectory.target_state.speed;
    // check if we need to drive behind a vehicle
    preceding_vehicle_id = this->map.find_next_vehicle_in_lane(car.frenet.s, trajectory.target_lane, sensor_fusion);
    preceding_vehicle_iter = std::find_if(sensor_fusion.begin(), sensor_fusion.end(), [&preceding_vehicle_id](const VehicleState &vehicle) { return preceding_vehicle_id == vehicle.id; });
    if (preceding_vehicle_iter != sensor_fusion.end()
        && this->map.get_frenet_s_distance_from_to(car.frenet.s, preceding_vehicle_iter->frenet.s) < this->max_speed * time_horizon) {
        proposed_speed_final = preceding_vehicle_iter->speed;
    }

    return logistic((2.0 * this->max_speed - proposed_speed_intended - proposed_speed_final) / this->max_speed);
}

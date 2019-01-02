//
// Created by chmst on 10/11/2016.
//

#include "BehaviorPlanner.h"


Behavior BehaviorPlanner::plan_next_behavior(const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) {
    if (this->lane == -1000) {
        this->lane = this->map.GetLaneFrom(car.frenet);
        log(2) << "set initial lane to " << this->lane << std::endl;
    }

    // only consider states which can be reached from current state
    auto possible_successor_states = this->successor_states();
    log(2) << "possible successor states:";
    for (auto state : possible_successor_states) {
        switch (state) {
            case BehaviorState::ConstantSpeed:
                log(2) << " ConstantSpeed";
                break;
            case BehaviorState::KeepLane:
                log(2) << " KeepLane";
                break;
            case BehaviorState::PrepareLaneChangeLeft:
                log(2) << " PrepareLaneChangeLeft";
                break;
            case BehaviorState::PrepareLaneChangeRight:
                log(2) << " PrepareLaneChangeRight";
                break;
            case BehaviorState::LaneChangeLeft:
                log(2) << " LaneChangeLeft";
                break;
            case BehaviorState::LaneChangeRight:
                log(2) << " LaneChangeRight";
                break;
        }
    }
    log(2) << std::endl;

    // find the minimum cost state
    auto best_next_state = BehaviorState::KeepLane;
    auto trajectory_for_best_state = this->generate_trajectory(best_next_state, car, time_horizon, sensor_fusion);
    float min_cost = 9999999;
    for (auto state : possible_successor_states) {
        // generate a rough idea of what trajectory we would
        // follow if we chose this state
        log(2) << "generate trajectory" << std::endl;
        auto trajectory_for_state = this->generate_trajectory(state, car, time_horizon, sensor_fusion);
        if (trajectory_for_state.is_trajectory_possible) {
            // calculate the "cost" associated with that trajectory
            auto cost_for_state = this->cost.calculate_cost(trajectory_for_state, car, timestep, time_horizon, sensor_fusion);
            log(2) << "cost: " << cost_for_state << std::endl;
            if (cost_for_state < min_cost) {
                min_cost = cost_for_state;
                best_next_state = state;
                trajectory_for_best_state = trajectory_for_state;
            }
        }
    }

    Behavior behavior;
    behavior.lane = this->map.GetLaneFrom(trajectory_for_best_state.target_state.frenet);
    behavior.max_speed = trajectory_for_best_state.target_state.speed;
    behavior.min_safety_zone_time = this->min_safety_zone_time;
    behavior.vehicle_id = trajectory_for_best_state.preceding_vehicle_id;
    return behavior;
}

TrajectoryKinematics BehaviorPlanner::try_follow_vehicle(const VehicleState &car, const int vehicle_id, const int target_lane, const int intended_lane, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    auto acceleration = (this->max_speed - car.speed) / time_horizon;
    if (acceleration > this->max_acceleration) {
        acceleration = this->max_acceleration;
    } else if (acceleration < -this->max_acceleration) {
        acceleration = -this->max_acceleration;
    }
    auto target_speed = car.speed + acceleration * time_horizon;
    // vehicle position, in the future
    auto future_s = car.frenet.s + car.speed * time_horizon + 0.5 * acceleration * pow<2>(time_horizon);

    int preceding_vehicle_id = -1;

    // check if we need to follow a vehicle in front of us
    auto preceding_vehicle_iter = std::find_if(sensor_fusion.begin(), sensor_fusion.end(), [&vehicle_id](const VehicleState &vehicle) { return vehicle_id == vehicle.id; });
    if (preceding_vehicle_iter != sensor_fusion.end()) {
        auto preceding_vehicle = *preceding_vehicle_iter;
        // preceding vehicle position, in the future
        auto preceding_vehicle_prediction = this->map.PredictIntoFuture(preceding_vehicle, time_horizon);
        // minimum distance to vehicle in front of us, if we drive with desired target speed
        auto min_distance_with_max_speed = this->min_safety_zone_time * this->max_speed;
        // calculate s for both (the preceding vehicle in the future and me in the future if I drive with the desired target speed)
        // calculate s relative to my current car position (combats the fact, that s jumps when I drive over the starting line (s=0 wraparound))
        auto rel_s_preceding_vehicle_prediction = this->map.GetFrenetSDistanceFromTo(car.frenet.s, preceding_vehicle_prediction.frenet.s);
        auto rel_s_target = this->map.GetFrenetSDistanceFromTo(car.frenet.s, future_s);
        auto actual_distance_to_preceding_vehicle = rel_s_preceding_vehicle_prediction - rel_s_target;
        if (actual_distance_to_preceding_vehicle < min_distance_with_max_speed) {
            // to fast -> no safety zone -> drive with speed of preceding vehicle and with safety zone
            // calculate acceleration, needed to keep minimum distance to preceding vehicle
            acceleration = (preceding_vehicle_prediction.frenet.s - car.frenet.s - car.speed * (time_horizon + this->min_safety_zone_time))
                / (0.5 * pow<2>(time_horizon) + time_horizon * this->min_safety_zone_time);
            if (acceleration > this->max_acceleration) {
                acceleration = this->max_acceleration;
            } else if (acceleration < -this->max_acceleration) {
                acceleration = -this->max_acceleration;
            }
            target_speed = car.speed + acceleration * time_horizon;
            // vehicle position, in the future
            future_s = car.frenet.s + car.speed * time_horizon + 0.5 * acceleration * pow<2>(time_horizon);
            preceding_vehicle_id = preceding_vehicle.id;
        }
    }

    FrenetCoordinate frenet_target(future_s, this->map.GetFrenetDFromLane(this->lane));
    auto cartesian_target = this->map.ConvertToCartesianPosition(frenet_target);
    VehicleState target_state(-1, cartesian_target, frenet_target, target_speed);
    return TrajectoryKinematics(car, this->lane, true, target_state, target_lane, intended_lane, acceleration, time_horizon, preceding_vehicle_id);
}

std::vector<BehaviorState> BehaviorPlanner::successor_states() const {
    std::vector<BehaviorState> states;
    states.push_back(BehaviorState::KeepLane);
    switch (this->state) {
        case BehaviorState::ConstantSpeed:
            break;
        case BehaviorState::KeepLane:
            if (this->lane > 0) {
                states.push_back(BehaviorState::PrepareLaneChangeLeft);
            }
            if (this->lane < this->max_lanes - 1) {
                states.push_back(BehaviorState::PrepareLaneChangeRight);
            }
            break;
        case BehaviorState::PrepareLaneChangeLeft:
            if (this->lane > 0) {
                states.push_back(BehaviorState::PrepareLaneChangeLeft);
                states.push_back(BehaviorState::LaneChangeLeft);
            }
            break;
        case BehaviorState::PrepareLaneChangeRight:
            if (this->lane < this->max_lanes - 1) {
                states.push_back(BehaviorState::PrepareLaneChangeRight);
                states.push_back(BehaviorState::LaneChangeRight);
            }
            break;
        case BehaviorState::LaneChangeLeft:
            break;
        case BehaviorState::LaneChangeRight:
            break;
    }

    return states;
}

TrajectoryKinematics BehaviorPlanner::generate_trajectory(const BehaviorState state, const VehicleState &car, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    switch (this->state) {
        case BehaviorState::ConstantSpeed:
            return this->constant_speed_trajectory(car, time_horizon);
            break;
        case BehaviorState::KeepLane:
            return this->keep_lane_trajectory(car, time_horizon, sensor_fusion);
            break;
        case BehaviorState::PrepareLaneChangeLeft:
        case BehaviorState::PrepareLaneChangeRight:
            return this->prep_lane_change_trajectory(state, car, time_horizon, sensor_fusion);
            break;
        case BehaviorState::LaneChangeLeft:
        case BehaviorState::LaneChangeRight:
            return this->lane_change_trajectory(state, car, time_horizon, sensor_fusion);
            break;
        default:
            // this should never happen
            return TrajectoryKinematics(car, this->lane, false, car, this->lane, this->lane, 0_m / 1_s / 1_s, 0_s, -1);
    }
}

TrajectoryKinematics BehaviorPlanner::constant_speed_trajectory(const VehicleState &car, const Time time_horizon) const {
    /*
    Generate a constant speed trajectory.
    */
    return TrajectoryKinematics(
        car,
        this->lane,
        true,
        this->map.PredictIntoFuture(car, time_horizon),
        this->lane,
        this->lane,
        0_m / 1_s / 1_s,
        time_horizon,
        -1);
}

TrajectoryKinematics BehaviorPlanner::keep_lane_trajectory(const VehicleState &car, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    /*
    Generate a keep lane trajectory.
    */
    auto vehicle_id = this->map.find_next_vehicle_in_lane(car.frenet.s, this->lane, time_horizon, sensor_fusion);
    return this->try_follow_vehicle(car, vehicle_id, this->lane, this->lane, time_horizon, sensor_fusion);
}

TrajectoryKinematics BehaviorPlanner::prep_lane_change_trajectory(const BehaviorState state, const VehicleState &car, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    /*
    Generate a trajectory preparing for a lane change.
    */
    int new_lane = this->lane;
    switch (state) {
        case BehaviorState::PrepareLaneChangeLeft:
        case BehaviorState::LaneChangeLeft:
            --new_lane;
            break;
        case BehaviorState::PrepareLaneChangeRight:
        case BehaviorState::LaneChangeRight:
            ++new_lane;
            break;
        default:
            break;
    }

    auto vehicle_id_current_lane = this->map.find_next_vehicle_in_lane(car.frenet.s, this->lane, time_horizon, sensor_fusion);
    auto curr_lane_new_kinematics = this->try_follow_vehicle(car, vehicle_id_current_lane, this->lane, new_lane, time_horizon, sensor_fusion);

    auto vehicle_id_new_lane = this->map.find_next_vehicle_in_lane(car.frenet.s, new_lane, time_horizon, sensor_fusion);
    auto new_lane_new_kinematics = this->try_follow_vehicle(car, vehicle_id_new_lane, this->lane, new_lane, time_horizon, sensor_fusion);

    // choose kinematics with lowest velocity.
    if (new_lane_new_kinematics.target_state.speed < curr_lane_new_kinematics.target_state.speed) {
        return new_lane_new_kinematics;
    } else {
        return curr_lane_new_kinematics;
    }
}

TrajectoryKinematics BehaviorPlanner::lane_change_trajectory(const BehaviorState state, const VehicleState &car, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) const {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane;
    switch (state) {
        case BehaviorState::PrepareLaneChangeLeft:
        case BehaviorState::LaneChangeLeft:
            --new_lane;
            break;
        case BehaviorState::PrepareLaneChangeRight:
        case BehaviorState::LaneChangeRight:
            ++new_lane;
            break;
        default:
            break;
    }

    TrajectoryKinematics error_trajectory(car, this->lane, false, car, this->lane, this->lane, 0_m / 1_s / 1_s, 0_s, -1);

    // check if a lane change is possible (check if another vehicle occupies that spot)
    auto vehicle_id_current_lane = this->map.find_next_vehicle_in_lane(car.frenet.s, this->lane, time_horizon, sensor_fusion);
    auto curr_lane_new_kinematics = this->try_follow_vehicle(car, vehicle_id_current_lane, new_lane, new_lane, time_horizon, sensor_fusion);

    auto vehicle_id_new_lane = this->map.find_next_vehicle_in_lane(car.frenet.s, new_lane, time_horizon, sensor_fusion);
    auto new_lane_new_kinematics = this->try_follow_vehicle(car, vehicle_id_new_lane, new_lane, new_lane, time_horizon, sensor_fusion);

    // if any trajectory not possible
    if (!curr_lane_new_kinematics.is_trajectory_possible
        || !new_lane_new_kinematics.is_trajectory_possible) {
        return error_trajectory;
    }

    // choose kinematics with lowest velocity
    auto combined_trajectory = new_lane_new_kinematics;
    if (curr_lane_new_kinematics.target_state.speed < new_lane_new_kinematics.target_state.speed) {
        combined_trajectory = curr_lane_new_kinematics;
    }
    // set target state to new lane
    combined_trajectory.target_state.frenet.d = this->map.GetFrenetDFromLane(new_lane);
    combined_trajectory.target_state.cartesian = this->map.ConvertToCartesianPosition(combined_trajectory.target_state.frenet);

    // check position of all other vehicles
    for (auto it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
        auto other_vehicle_lane = this->map.GetLaneFrom(it->frenet);
        if (other_vehicle_lane == new_lane) {
            // check if it is save to change lanes (no collision)
            auto t_step = std::min(1_s, std::max(0.02_s, 2_m / this->max_speed));
            // check the time interval [0; time_horizon]
            for (auto t = 0_s; t <= time_horizon + 0.001_s; t += t_step) {
                auto vehicle_prediction = this->map.PredictIntoFuture(*it, t);
                auto self_prediction = this->evaluate_trajectory(combined_trajectory, t);
                auto vehicle_min_safety_zone = vehicle_prediction.frenet.s + this->lane_change_min_safety_zone_time * vehicle_prediction.speed;
                auto self_min_safety_zone = self_prediction.frenet.s + this->lane_change_min_safety_zone_time * self_prediction.speed;

                // each vehicle has it's own safety zone in front of it
                //     +-----------+                               |
                //     |     +  >> | <~ ~ ~ ~ safety zone ~ ~ ~ ~> |
                //     +-----------+                               |
                if (self_prediction.frenet.s >= vehicle_prediction.frenet.s
                    && self_prediction.frenet.s <= vehicle_min_safety_zone) {
                    // violate other vehicle's safety zone
                    return error_trajectory;
                }
                if (vehicle_prediction.frenet.s >= self_prediction.frenet.s
                    && vehicle_prediction.frenet.s <= self_min_safety_zone) {
                    // violate my safety zone
                    return error_trajectory;
                }
            }
        }
    }

    return combined_trajectory;
}

#include "BehaviorPlanner.h"
#include <algorithm>
#include "log.h"


Behavior BehaviorPlanner::plan_next_behavior(const VehicleState &car, const Time timestep, const Time time_horizon, const std::vector<VehicleState> &sensor_fusion) {
    auto logger = LogLevelStack(1);
    auto now = std::chrono::system_clock::now();

    if (this->lane == -1000) {
        this->lane = this->map.get_lane_from(car.frenet);
        log(3) << "set initial lane to " << this->lane << std::endl;
    }

    auto current_trajectory = this->generate_trajectory(this->state, car, time_horizon, sensor_fusion);

    Behavior behavior;
    if (now - this->last_state_change > this->min_state_time) {
        this->last_state_change = now;

        // set lane to target lane, only if the next state will be choosen
        this->lane = current_trajectory.target_lane;

        log(2) << std::endl;
        log(2) << "Behavior:" << std::endl;
        log(2) << "---------" << std::endl;

        // only consider states which can be reached from current state
        log(2) << "current state: " << this->state << " " << car.frenet << std::endl;
        auto possible_successor_states = this->successor_states();
        log(3) << "possible successor states:";
        for (auto state : possible_successor_states) {
            log(3) << " " << state;
        }
        log(3) << std::endl;

        // find the minimum cost state
        auto best_next_state = BehaviorState::KeepLane;
        auto trajectory_for_best_state = this->generate_trajectory(best_next_state, car, time_horizon, sensor_fusion);
        float min_cost = 9999999;
        for (auto state : possible_successor_states) {
            // generate a rough idea of what trajectory we would
            // follow if we chose this state
            auto trajectory_for_state = this->generate_trajectory(state, car, time_horizon, sensor_fusion);
            if (trajectory_for_state.is_trajectory_possible) {
                // calculate the "cost" associated with that trajectory
                auto cost_for_state = this->cost.calculate_cost(trajectory_for_state, car, timestep, time_horizon, sensor_fusion);
                log(3) << "cost for " << state << ": " << cost_for_state << std::endl;
                if (cost_for_state < min_cost) {
                    min_cost = cost_for_state;
                    best_next_state = state;
                    trajectory_for_best_state = trajectory_for_state;
                }
            }
        }

        behavior.lane = trajectory_for_best_state.target_lane;
        behavior.max_speed = this->max_speed;
        behavior.min_safety_zone_time = this->min_safety_zone_time;
        behavior.vehicle_id = trajectory_for_best_state.preceding_vehicle_id;

        if (this->state != best_next_state) {
            this->state = best_next_state;
            log(1) << "new state: " << this->state << std::endl;
        }

        // output behavior
        log(3) << "next state: " << this->state << std::endl;
        log(2) << "lane: " << behavior.lane << std::endl;
        log(2) << "max speed: " << behavior.max_speed << std::endl;
        log(2) << "min safety zone time: " << behavior.min_safety_zone_time << std::endl;
        log(2) << "vehicle id: " << behavior.vehicle_id << std::endl;
    } else {
        behavior.lane = current_trajectory.target_lane;
        behavior.max_speed = this->max_speed;
        behavior.min_safety_zone_time = this->min_safety_zone_time;
        behavior.vehicle_id = current_trajectory.preceding_vehicle_id;
    }

#if PLOTSIGNALS
    log_signal("state", (int)this->state);
    log_signal("planner lane", this->lane);
    log_signal("behavior lane", behavior.lane);
    log_signal("target vehicle", behavior.vehicle_id);
    log_signal("max speed", behavior.max_speed.value);
    auto acceleration = (behavior.max_speed - car.speed) / time_horizon;
    log_signal("acceleration", acceleration.value);
#endif

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
    auto target_distance = car.speed * time_horizon + 0.5 * acceleration * pow<2>(time_horizon);
    // vehicle position, in the future
    auto frenet_target = this->map.add_lane_distance(
        car.frenet,
        target_distance,
        this->map.get_frenet_d_from_lane(this->lane));
    auto cartesian_target = this->map.convert_to_cartesian_position(frenet_target);

    int preceding_vehicle_id = -1;

    // check if we need to follow a vehicle in front of us
    auto preceding_vehicle_iter = std::find_if(sensor_fusion.begin(), sensor_fusion.end(), [&vehicle_id](const VehicleState &vehicle) { return vehicle_id == vehicle.id; });
    if (preceding_vehicle_iter != sensor_fusion.end()) {
        auto preceding_vehicle = *preceding_vehicle_iter;
        // preceding vehicle position, in the future
        auto preceding_vehicle_prediction = this->map.predict_into_future_onto_lane(preceding_vehicle, time_horizon);
        // minimum distance to vehicle in front of us, if we drive with desired target speed
        auto min_distance_with_max_speed = this->min_safety_zone_time * this->max_speed;
        // calculate s for both (the preceding vehicle in the future and me in the future if I drive with the desired target speed)
        // calculate s relative to my current car position (combats the fact, that s jumps when I drive over the starting line (s=0 wraparound))
        auto rel_dist_preceding_vehicle_prediction = this->map.get_lane_distance_from_to(car.frenet, preceding_vehicle_prediction.frenet);
        auto rel_dist_target = this->map.get_lane_distance_from_to(car.frenet, frenet_target);
        auto actual_distance_to_preceding_vehicle = rel_dist_preceding_vehicle_prediction - rel_dist_target;
        if (actual_distance_to_preceding_vehicle < min_distance_with_max_speed) {
            // to fast -> no safety zone -> drive with speed of preceding vehicle and with safety zone
            // calculate acceleration, needed to keep minimum distance to preceding vehicle
            acceleration = (rel_dist_preceding_vehicle_prediction - car.speed * (time_horizon + this->min_safety_zone_time))
                / (0.5 * pow<2>(time_horizon) + time_horizon * this->min_safety_zone_time);
            if (acceleration > this->max_acceleration) {
                acceleration = this->max_acceleration;
            } else if (acceleration < -this->max_acceleration) {
                acceleration = -this->max_acceleration;
            }
            target_speed = car.speed + acceleration * time_horizon;
            if (target_speed < 0.1_m / 1_s) {
                target_speed = 0.1_m / 1_s;
                acceleration = (target_speed - car.speed) / time_horizon;
            }
            target_distance = car.speed * time_horizon + 0.5 * acceleration * pow<2>(time_horizon);
            // vehicle position, in the future
            frenet_target = this->map.add_lane_distance(
                car.frenet,
                target_distance,
                this->map.get_frenet_d_from_lane(this->lane));
            cartesian_target = this->map.convert_to_cartesian_position(frenet_target);
            preceding_vehicle_id = preceding_vehicle.id;
        }
    }

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
    switch (state) {
        case BehaviorState::ConstantSpeed:
            return this->constant_speed_trajectory(car, time_horizon);
        case BehaviorState::KeepLane:
            return this->keep_lane_trajectory(car, time_horizon, sensor_fusion);
        case BehaviorState::PrepareLaneChangeLeft:
        case BehaviorState::PrepareLaneChangeRight:
            return this->prep_lane_change_trajectory(state, car, time_horizon, sensor_fusion);
        case BehaviorState::LaneChangeLeft:
        case BehaviorState::LaneChangeRight:
            return this->lane_change_trajectory(state, car, time_horizon, sensor_fusion);
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
        this->map.predict_into_future_in_cartesian(car, time_horizon),
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
    auto vehicle_id = this->map.find_next_vehicle_in_lane(car.frenet.s, this->lane, sensor_fusion);
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

    TrajectoryKinematics error_trajectory(car, this->lane, false, car, this->lane, this->lane, 0_m / 1_s / 1_s, 0_s, -1);

    auto vehicle_id_current_lane = this->map.find_next_vehicle_in_lane(car.frenet.s, this->lane, sensor_fusion);
    auto curr_lane_new_kinematics = this->try_follow_vehicle(car, vehicle_id_current_lane, this->lane, new_lane, time_horizon, sensor_fusion);

    auto look_behind = (this->min_safety_zone_time * this->max_speed) / 3;
    auto vehicle_id_new_lane = this->map.find_next_vehicle_in_lane(car.frenet.s - look_behind, new_lane, sensor_fusion);
    auto new_lane_new_kinematics = this->try_follow_vehicle(car, vehicle_id_new_lane, this->lane, new_lane, time_horizon, sensor_fusion);

    // if any trajectory not possible
    if (!curr_lane_new_kinematics.is_trajectory_possible
        || !new_lane_new_kinematics.is_trajectory_possible) {
        return error_trajectory;
    }

    // choose kinematics with lowest velocity.
    // ToDo: may also check distance to vehicle
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

    auto vehicle_id_current_lane = this->map.find_next_vehicle_in_lane(car.frenet.s, this->lane, sensor_fusion);
    auto curr_lane_new_kinematics = this->try_follow_vehicle(car, vehicle_id_current_lane, new_lane, new_lane, time_horizon, sensor_fusion);

    auto look_behind = (this->min_safety_zone_time * this->max_speed) / 3;
    auto vehicle_id_new_lane = this->map.find_next_vehicle_in_lane(car.frenet.s - look_behind, new_lane, sensor_fusion);
    auto new_lane_new_kinematics = this->try_follow_vehicle(car, vehicle_id_new_lane, new_lane, new_lane, time_horizon, sensor_fusion);

    // if any trajectory not possible
    if (!curr_lane_new_kinematics.is_trajectory_possible
        || !new_lane_new_kinematics.is_trajectory_possible) {
        return error_trajectory;
    }

    // choose kinematics with lowest velocity
    // ToDo: may also check distance to vehicle
    auto combined_trajectory = new_lane_new_kinematics;
    if (curr_lane_new_kinematics.target_state.speed < new_lane_new_kinematics.target_state.speed) {
        combined_trajectory = curr_lane_new_kinematics;
    }

    // set target state to new lane
    combined_trajectory.target_state.frenet.d = this->map.get_frenet_d_from_lane(new_lane); // maybe adjust it, because the trajectory planner uses a fixed distance of 30m in front of the current position, to get to the new lane
    combined_trajectory.target_state.cartesian = this->map.convert_to_cartesian_position(combined_trajectory.target_state.frenet);

    return combined_trajectory;
}

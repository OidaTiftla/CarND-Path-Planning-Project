//
// Created by chmst on 10/11/2016.
//

#include "TrajectoryPlanner.h"


std::vector<GlobalCartesianCoordinate> TrajectoryPlanner::PlanNextTrajectory(const Behavior& behavior, const Time timestep, const Time time_horizon) const {
    log(1) << std::endl;
    log(1) << "Target state:" << std::endl;
    log(1) << "-------------" << std::endl;
    log(1) << "timestep: " << timestep << std::endl;
    log(1) << "time horizon: " << time_horizon << std::endl;

    // reuse some previous points
    int count_previous = ceil(time_horizon * 0.2 / timestep);
    count_previous = std::min(count_previous, (int)this->previous_path.size());
    // are there enough previous points?
    if (count_previous < 3) {
        count_previous = 0;
    }

    // set start state
    auto start_state = this->car;
    auto remaining_time_horizon = time_horizon;
    if (count_previous >= 3) {
        auto last = this->previous_path[count_previous - 1];
        auto last2 = this->previous_path[count_previous - 2];
        GlobalCartesianPosition pos(last.x, last.y, last2.AngleTo(last));
        auto frenet = this->map.ConvertToFrenet(pos);
        auto speed = last2.DistanceTo(last) / timestep;
        start_state = VehicleState(-1, pos, frenet, speed);
        remaining_time_horizon = time_horizon - count_previous * timestep;
    }
    log(1) << "start: " << start_state.frenet << ", " << start_state.speed << std::endl;
    log(1) << "remaining time horizon: " << remaining_time_horizon << std::endl;

    auto acceleration = (behavior.max_speed - start_state.speed) / remaining_time_horizon;
    if (acceleration > this->max_acceleration) {
        log(1) << "exceed max acceleration" << std::endl;
        acceleration = this->max_acceleration;
    } else if (acceleration < -this->max_acceleration) {
        log(1) << "exceed max deceleration" << std::endl;
        acceleration = -this->max_acceleration;
    }
    auto target_speed = start_state.speed + acceleration * remaining_time_horizon;
    FrenetCoordinate target_frenet(
        start_state.frenet.s + start_state.speed * remaining_time_horizon + 0.5 * acceleration * pow<2>(remaining_time_horizon),
        this->map.GetFrenetDFromLane(behavior.lane));

    auto preceding_vehicle_iter = std::find_if(this->sensor_fusion.begin(), this->sensor_fusion.end(), [&behavior] (const VehicleState& vehicle) { return behavior.vehicle_id == vehicle.id; } );
    if (preceding_vehicle_iter != this->sensor_fusion.end()) {
        auto preceding_vehicle = *preceding_vehicle_iter;
        // preceding vehicle position, in the future
        auto preceding_vehicle_prediction = this->map.PredictIntoFuture(preceding_vehicle, time_horizon);
        // minimum distance to vehicle in front of us, if we drive with desired target speed
        auto min_distance_with_target_speed = behavior.min_safety_zone_time * target_speed;
        // calculate s for both (the preceding vehicle in the future and me in the future if I drive with the desired target speed)
        // calculate s relative to my current car position (combats the fact, that s jumps when I drive over the starting line (s=0 wraparound))
        auto rel_s_preceding_vehicle_prediction = this->map.GetFrenetSDistanceFromTo(this->car.frenet.s, preceding_vehicle_prediction.frenet.s);
        auto rel_s_target = this->map.GetFrenetSDistanceFromTo(this->car.frenet.s, target_frenet.s);
        if (rel_s_preceding_vehicle_prediction - min_distance_with_target_speed < rel_s_target) {
            // to fast -> no safety zone -> drive with speed of preceding vehicle and with safety zone
            log(1) << "too fast for vehicle in front of us (s prediction: " << preceding_vehicle_prediction.frenet.s << ")" << std::endl;
            // set target speed to speed of vehicle in front of us
            target_speed = preceding_vehicle_prediction.speed;
            // and set the target s value to the s value of the preceding vehicle
            // minus the minimum distance to vehicle in front of us, if we drive with its speed
            auto min_distance_with_preceding_vehicle_speed = behavior.min_safety_zone_time * preceding_vehicle_prediction.speed;
            target_frenet.s = this->map.NormalizeS(preceding_vehicle_prediction.frenet.s - min_distance_with_preceding_vehicle_speed);

            // validate speed
            if (target_speed > behavior.max_speed) {
                log(1) << "exceed max speed (preceding vehicle)" << std::endl;
                target_speed = behavior.max_speed;
            } else if (target_speed < 0_m / 1_s) {
                log(1) << "exceed max backwards speed (preceding vehicle)" << std::endl;
                target_speed = 0_m / 1_s;
            }

            // calculate resulting acceleration
            acceleration = (target_speed - start_state.speed) / remaining_time_horizon;

            // validate acceleration
            if (acceleration > this->max_acceleration) {
                log(1) << "exceed max acceleration (preceding vehicle)" << std::endl;
                acceleration = this->max_acceleration;
            } else if (acceleration < -this->max_acceleration) {
                log(1) << "exceed max deceleration (preceding vehicle)" << std::endl;
                acceleration = -this->max_acceleration;
            }

            // validate target state based on the physics of the car
            auto calc_target_speed = start_state.speed + acceleration * remaining_time_horizon;
            if (abs(target_speed - calc_target_speed) > 0.1_m / 1_s) {
                // target speed not matching the calculations -> target speed not possible
                // correct it (based on the physics of the car)
                log(1) << "update target speed from " << target_speed  << " to " << calc_target_speed  << std::endl;
                target_speed = calc_target_speed;
            }
            auto calc_s = start_state.frenet.s + start_state.speed * remaining_time_horizon + 0.5 * acceleration * pow<2>(remaining_time_horizon);
            if (abs(target_frenet.s - calc_s) > 0.1_m) {
                // target s not matching the calculations -> target s not possible
                // correct it (based on the physics of the car)
                log(1) << "update target s from " << target_frenet.s  << " to " << calc_s  << std::endl;
                target_frenet.s = calc_s;
            }
        }
    }

    auto target_cartesian = this->map.ConvertToCartesian(target_frenet);
    auto target_cartesian_5m = this->map.ConvertToCartesian(FrenetCoordinate(target_frenet.s + 5_m, target_frenet.d));
    auto target_theta = target_cartesian.AngleTo(target_cartesian_5m);
    VehicleState target_state(-1, GlobalCartesianPosition(target_cartesian.x, target_cartesian.y, target_theta), target_frenet, target_speed);

    // output target state
    log(1) << "s: " << target_state.frenet.s << std::endl;
    log(1) << "d: " << target_state.frenet.d << std::endl;
    log(1) << "speed: " << target_state.speed << std::endl;
    log(1) << "acceleration: " << (target_state.speed - start_state.speed) / remaining_time_horizon << std::endl;
    log(1) << "x: " << target_state.cartesian.coord.x << std::endl;
    log(1) << "y: " << target_state.cartesian.coord.y << std::endl;
    log(1) << "theta: " << ToDegree(target_state.cartesian.theta) << std::endl;
    log(1) << "speed x: " << target_state.speed_x << std::endl;
    log(1) << "speed y: " << target_state.speed_y << std::endl;

    return this->CalculateTrajectory(count_previous, target_state, timestep, time_horizon);
}

std::vector<GlobalCartesianCoordinate> TrajectoryPlanner::CalculateTrajectory(const int count_previous, const VehicleState& target_state, const Time timestep, const Time time_horizon) const {
    CoordinateSystemReference local_system(this->car.cartesian);

    log(2) << std::endl;
    log(2) << "Calculate JMT:" << std::endl;
    log(2) << "--------------" << std::endl;

    State start_x, start_y, end_x, end_y;

    // define starting state
    auto remaining_time_horizon = time_horizon;
    LocalCartesianCoordinate wp_last, wp_last2, wp_last3;
    if (count_previous >= 3) {
        remaining_time_horizon = time_horizon - count_previous * timestep;
        log(2) << "create starting state from last 3 waypoints" << std::endl;
        wp_last = local_system.ToLocal(this->previous_path[count_previous - 1]);
        wp_last2 = local_system.ToLocal(this->previous_path[count_previous - 2]);
        wp_last3 = local_system.ToLocal(this->previous_path[count_previous - 3]);
    } else {
        log(2) << "create starting state from current position" << std::endl;
        wp_last = local_system.ToLocal(this->car.cartesian.coord);
        wp_last2 = local_system.ToLocal(this->map.PredictIntoFuture(this->car, -timestep).cartesian.coord);
        wp_last3 = local_system.ToLocal(this->map.PredictIntoFuture(this->car, -timestep * 2).cartesian.coord);
    }
    auto wp_last_v_x = (wp_last.x - wp_last2.x) / timestep;
    auto wp_last_v_y = (wp_last.y - wp_last2.y) / timestep;
    auto wp_last_v2_x = (wp_last2.x - wp_last3.x) / timestep;
    auto wp_last_v2_y = (wp_last2.y - wp_last3.y) / timestep;
    start_x.x = wp_last.x;
    start_y.x = wp_last.y;
    start_x.x_dot = wp_last_v_x;
    start_y.x_dot = wp_last_v_y;
    start_x.x_dot_dot = (wp_last_v_x - wp_last_v2_x) / timestep;
    start_y.x_dot_dot = (wp_last_v_y - wp_last_v2_y) / timestep;
    log(2) << "starting state:" << std::endl;
    log(2) << "x: " << start_x.x << std::endl;
    log(2) << "y: " << start_y.x << std::endl;
    log(2) << "speed x: " << start_x.x_dot << std::endl;
    log(2) << "speed y: " << start_y.x_dot << std::endl;
    log(2) << "acceleration x: " << start_x.x_dot_dot << std::endl;
    log(2) << "acceleration y: " << start_y.x_dot_dot << std::endl;

    // define end state
    auto future_time_for_direction = 10 * timestep;
    auto wp_target = local_system.ToLocal(target_state.cartesian.coord);
    auto wp_target2 = local_system.ToLocal(this->map.PredictIntoFuture(target_state, future_time_for_direction).cartesian.coord);
    end_x.x = wp_target.x;
    end_y.x = wp_target.y;
    end_x.x_dot = (wp_target2.x - wp_target.x) / future_time_for_direction; // speed in local coordinate system
    end_y.x_dot = (wp_target2.y - wp_target.y) / future_time_for_direction; // speed in local coordinate system
    end_x.x_dot_dot = 0_m / 1_s / 1_s;
    end_y.x_dot_dot = 0_m / 1_s / 1_s;
    log(2) << "end state:" << std::endl;
    log(2) << "x: " << end_x.x << std::endl;
    log(2) << "y: " << end_y.x << std::endl;
    log(2) << "speed x: " << end_x.x_dot << std::endl;
    log(2) << "speed y: " << end_y.x_dot << std::endl;
    log(2) << "acceleration x: " << end_x.x_dot_dot << std::endl;
    log(2) << "acceleration y: " << end_y.x_dot_dot << std::endl;

    // calculate JMT
    auto coeffs_x = JMT(start_x, end_x, remaining_time_horizon);
    auto coeffs_y = JMT(start_y, end_y, remaining_time_horizon);
    log(2) << "coeffs:" << std::endl;
    log(2) << "x: "
        << coeffs_x.a_0 << ", "
        << coeffs_x.a_1 << ", "
        << coeffs_x.a_2 << ", "
        << coeffs_x.a_3 << ", "
        << coeffs_x.a_4 << ", "
        << coeffs_x.a_5 << ", "
        << std::endl;
    log(2) << "y: "
        << coeffs_y.a_0 << ", "
        << coeffs_y.a_1 << ", "
        << coeffs_y.a_2 << ", "
        << coeffs_y.a_3 << ", "
        << coeffs_y.a_4 << ", "
        << coeffs_y.a_5 << ", "
        << std::endl;

    log(2) << std::endl;
    log(2) << "Calculate trajectory:" << std::endl;
    log(2) << "---------------------" << std::endl;
    // interpolate polynomial
    std::vector<GlobalCartesianCoordinate> trajectory;
    int count = ceil(time_horizon / timestep);
    auto last = local_system.ToLocal(this->car.cartesian.coord);
    auto last_speed = this->car.speed;
    log(2) << "current car " << last << " with speed " << last_speed << std::endl;
    log(2) << "create " << count << " waypoints" << std::endl;
    for (int i = 0; i < count; ++i) {
        if (i < count_previous) {
            auto next = local_system.ToLocal(this->previous_path[i]);
            auto dist = last.DistanceTo(next);
            auto speed = dist / timestep;
            trajectory.push_back(this->previous_path[i]);
            last = next;
            last_speed = speed;
            log(2) << "reuse local waypoint " << next << " with speed " << speed << std::endl;
        } else {
            auto t = timestep * (i - count_previous + 1);
            auto x = coeffs_x.eval(t);
            auto y = coeffs_y.eval(t);
            auto x_dot = coeffs_x.eval_derivative(t);
            auto y_dot = coeffs_y.eval_derivative(t);
            auto x_dot_dot = coeffs_x.eval_derivative2(t);
            auto y_dot_dot = coeffs_y.eval_derivative2(t);
            auto acceleration = sqrt(pow<2>(x_dot_dot) + pow<2>(y_dot_dot));
            auto speed = sqrt(pow<2>(x_dot) + pow<2>(y_dot));
            auto next = LocalCartesianCoordinate(x, y);
            trajectory.push_back(local_system.ToGlobal(next));
            last = next;
            last_speed = speed;
            log(2) << "add local waypoint " << next << " with speed " << speed << " and acceleration " << acceleration << std::endl;
        }
    }

    return trajectory;
}

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
        auto frenet = this->map.ConvertToFrenet(pos.coord);
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
    log(2) << "Local s JMT:" << std::endl;
    log(2) << "------------" << std::endl;

    // define starting state
    auto remaining_time_horizon = time_horizon;
    LocalCartesianCoordinate wp_last, wp_last2, wp_last3;
    FrenetCoordinate wp_last_frenet;
    if (count_previous >= 3) {
        remaining_time_horizon = time_horizon - count_previous * timestep;
        log(2) << "create starting state from last 3 waypoints" << std::endl;
        wp_last = local_system.ToLocal(this->previous_path[count_previous - 1]);
        wp_last_frenet = this->map.ConvertToFrenet(this->previous_path[count_previous - 1]);
        wp_last2 = local_system.ToLocal(this->previous_path[count_previous - 2]);
        wp_last3 = local_system.ToLocal(this->previous_path[count_previous - 3]);
    } else {
        log(2) << "create starting state from current position" << std::endl;
        wp_last = local_system.ToLocal(this->car.cartesian.coord);
        wp_last_frenet = this->car.frenet;
        wp_last2 = local_system.ToLocal(this->map.PredictIntoFuture(this->car, -timestep).cartesian.coord);
        wp_last3 = local_system.ToLocal(this->map.PredictIntoFuture(this->car, -timestep * 2).cartesian.coord);
    }

    std::vector<LocalCartesianCoordinate> waypoints;
    auto total_s = target_state.frenet.s - wp_last_frenet.s;
    auto total_d = target_state.frenet.d - wp_last_frenet.d;
    auto count_segments = (int)ceil(total_s / 50_m);
    auto count_points = count_segments + 1;
    log(2) << "--- segments: " << count_segments << std::endl;
    for (int i = 0; i < count_points; ++i) {
        auto s = wp_last_frenet.s + total_s * i / count_segments;
        auto d = wp_last_frenet.d + total_d * i / count_segments;
        waypoints.push_back(local_system.ToLocal(this->map.ConvertToCartesian(FrenetCoordinate(s, d))));
    }
    // replace, because the calculations are not so exact, when converting into frenet coordinates
    waypoints[0] = wp_last;

    // correct total s in local system
    total_s = 0_m;
    for (int i = 0; i < count_segments; ++i) {
        total_s += waypoints[i].DistanceTo(waypoints[i + 1]);
    }

    auto wp_last_v = wp_last.DistanceTo(wp_last2) / timestep;
    auto wp_last_v2 = wp_last2.DistanceTo(wp_last3) / timestep;
    State start_s, end_s;
    start_s.x = 0_m;
    start_s.x_dot = wp_last_v;
    start_s.x_dot_dot = (wp_last_v - wp_last_v2) / timestep;

    auto wp_target = local_system.ToLocal(target_state.cartesian.coord);
    auto wp_target2 = local_system.ToLocal(this->map.PredictIntoFuture(target_state, timestep).cartesian.coord);
    end_s.x = total_s;
    end_s.x_dot = wp_target2.DistanceTo(wp_target) / timestep; // speed in local coordinate system
    end_s.x_dot_dot = 0_m / 1_s / 1_s;

    log(2) << "=== starting state:" << std::endl;
    log(2) << "s: " << start_s.x << std::endl;
    log(2) << "speed s: " << start_s.x_dot << std::endl;
    log(2) << "acceleration s: " << start_s.x_dot_dot << std::endl;
    log(2) << "=== end state:" << std::endl;
    log(2) << "s: " << end_s.x << std::endl;
    log(2) << "speed s: " << end_s.x_dot << std::endl;
    log(2) << "acceleration s: " << end_s.x_dot_dot << std::endl;

    // calculate JMT
    auto coeffs_s = JMT(start_s, end_s, remaining_time_horizon);
    log(2) << "coeffs :::::::::::::::::: s: "
        << coeffs_s.a_0 << ", "
        << coeffs_s.a_1 << ", "
        << coeffs_s.a_2 << ", "
        << coeffs_s.a_3 << ", "
        << coeffs_s.a_4 << ", "
        << coeffs_s.a_5 << ", "
        << std::endl;

    std::vector<State> segment_states_x, segment_states_y;
    auto dT_segment = remaining_time_horizon / count_segments;
    for (int i = 0; i < count_points; ++i) {
        auto t = i * dT_segment;
        // get distance, speed and acceleration from s JMT
        auto s = coeffs_s.eval(t);
        auto v = coeffs_s.eval_derivative(t);
        auto a = coeffs_s.eval_derivative2(t);

        // get x, y from s and also the angle, to calculate v_x and v_y (and a_x, a_y)
        auto current_s = 0_m;
        int k = 0;
        for (; k < count_segments; ++k) {
            if (current_s >= s) {
                break;
            }
            auto s_segment = waypoints[k].DistanceTo(waypoints[k + 1]);
            if (current_s + s_segment <= s) {
                current_s += s_segment;
            } else {
                break;
            }
        }
        auto angle = 0_rad;
        auto x = 0_m;
        auto y = 0_m;
        if (k >= count_segments) {
            // last point
            if (k >= 1) {
                angle = waypoints[k - 1].AngleTo(waypoints[k]);
            } else {
                angle = 0_rad;
            }
            x = waypoints[k].x;
            y = waypoints[k].y;
        } else {
            angle = waypoints[k].AngleTo(waypoints[k + 1]);
            auto s_segment = waypoints[k].DistanceTo(waypoints[k + 1]);
            auto part = (double)((s - current_s) / s_segment);
            if (k <= 0 && part < 0.001) {
                angle = 0_rad; // starting point
            }
            x = waypoints[k].x + (waypoints[k + 1].x - waypoints[k].x) * part;
            y = waypoints[k].y + (waypoints[k + 1].y - waypoints[k].y) * part;
        }

        State state_x, state_y;
        state_x.x = x;
        state_x.x_dot = v * cos(angle);
        state_x.x_dot_dot = a * cos(angle);
        state_y.x = y;
        state_y.x_dot = v * sin(angle);
        state_y.x_dot_dot = a * sin(angle);

        log(2) << "=== state:" << std::endl;
        log(2) << "x: " << state_x.x << std::endl;
        log(2) << "y: " << state_y.x << std::endl;
        log(2) << "speed x: " << state_x.x_dot << std::endl;
        log(2) << "speed y: " << state_y.x_dot << std::endl;
        log(2) << "acceleration x: " << state_x.x_dot_dot << std::endl;
        log(2) << "acceleration y: " << state_y.x_dot_dot << std::endl;

        segment_states_x.push_back(state_x);
        segment_states_y.push_back(state_y);
    }

    log(2) << std::endl;
    log(2) << "Calculate trajectory:" << std::endl;
    log(2) << "---------------------" << std::endl;
    // interpolate polynomial
    std::vector<GlobalCartesianCoordinate> trajectory;
    int count = ceil(time_horizon / timestep);
    auto last = local_system.ToLocal(this->car.cartesian.coord);
    auto last_speed = this->car.speed;
    auto i_segment = -1;
    Coefficients coeffs_x, coeffs_y;
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
            auto end_t_segment = (i_segment + 1) * dT_segment;
            if (t > end_t_segment) {
                // calculate next segment
                ++i_segment;
                auto start_x = segment_states_x[i_segment];
                auto end_x = segment_states_x[i_segment + 1];
                auto start_y = segment_states_y[i_segment];
                auto end_y = segment_states_y[i_segment + 1];

                // calculate JMT
                coeffs_x = JMT(start_x, end_x, dT_segment);
                coeffs_y = JMT(start_y, end_y, dT_segment);

                log(2) << "coeffs :::::::::::::::::: x: "
                    << coeffs_x.a_0 << ", "
                    << coeffs_x.a_1 << ", "
                    << coeffs_x.a_2 << ", "
                    << coeffs_x.a_3 << ", "
                    << coeffs_x.a_4 << ", "
                    << coeffs_x.a_5 << ", "
                    << std::endl;
                log(2) << "coeffs :::::::::::::::::: y: "
                    << coeffs_y.a_0 << ", "
                    << coeffs_y.a_1 << ", "
                    << coeffs_y.a_2 << ", "
                    << coeffs_y.a_3 << ", "
                    << coeffs_y.a_4 << ", "
                    << coeffs_y.a_5 << ", "
                    << std::endl;
            }
            auto start_t_segment = i_segment * dT_segment;
            auto t_seg = t - start_t_segment;

            auto x = coeffs_x.eval(t_seg);
            auto y = coeffs_y.eval(t_seg);
            auto x_dot = coeffs_x.eval_derivative(t_seg);
            auto y_dot = coeffs_y.eval_derivative(t_seg);
            auto x_dot_dot = coeffs_x.eval_derivative2(t_seg);
            auto y_dot_dot = coeffs_y.eval_derivative2(t_seg);
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

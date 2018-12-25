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
    if (count_previous < 2) {
        count_previous = 0;
    }

    // set start state
    auto start_state = this->car;
    auto remaining_time_horizon = time_horizon;
    if (count_previous >= 2) {
        auto last = this->previous_path[count_previous - 1];
        auto last2 = this->previous_path[count_previous - 2];
        GlobalCartesianPosition pos(last.x, last.y, last2.AngleTo(last));
        auto frenet = this->map.ConvertToFrenet(last);
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
        auto actual_distance_to_preceding_vehicle = rel_s_preceding_vehicle_prediction - rel_s_target;
        if (actual_distance_to_preceding_vehicle < min_distance_with_target_speed) {
            // to fast -> no safety zone -> drive with speed of preceding vehicle and with safety zone
            log(1) << "too fast for vehicle in front of us (s prediction: " << preceding_vehicle_prediction.frenet.s << ")" << std::endl;
            // set target speed to speed of vehicle in front of us
            target_speed = actual_distance_to_preceding_vehicle / behavior.min_safety_zone_time;
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
    log(2) << "Calculate spline:" << std::endl;
    log(2) << "-----------------" << std::endl;

    // spline control points
    std::vector<double> X, Y;

    // define starting points
    if (count_previous >= 2) {
        log(2) << "reuse waypoints" << std::endl;
        auto local1 = local_system.ToLocal(this->previous_path[0]);
        X.push_back(local1.x.value);
        Y.push_back(local1.y.value);
        log(2) << "reuse local waypoint " << local1 << std::endl;
        auto local2 = local_system.ToLocal(this->previous_path[count_previous - 1]);
        X.push_back(local2.x.value);
        Y.push_back(local2.y.value);
        log(2) << "reuse local waypoint " << local2 << std::endl;
    } else {
        log(2) << "create initial waypoints" << std::endl;

        auto speed_x = this->car.speed_x;
        auto speed_y = this->car.speed_y;
        if (this->car.speed <= 0.1_m / 1_s) {
            // current car has no speed
            // set new start speed
            auto speed = this->max_acceleration * timestep;
            speed_x = speed * cos(this->car.cartesian.theta);
            speed_y = speed * sin(this->car.cartesian.theta);
            log(2) << "set speed to " << speed << " (" << speed_x << ", " << speed_y << ")" << std::endl;
        }
        GlobalCartesianCoordinate cartesian_minus_0_1_target(
            this->car.cartesian.coord.x + (time_horizon * -0.1) * speed_x,
            this->car.cartesian.coord.y + (time_horizon * -0.1) * speed_y);
        auto local_minus_0_1_target = local_system.ToLocal(cartesian_minus_0_1_target);
        X.push_back(local_minus_0_1_target.x.value);
        Y.push_back(local_minus_0_1_target.y.value);
        log(2) << "add local waypoint " << local_minus_0_1_target << std::endl;

        auto local_current = local_system.ToLocal(this->car.cartesian.coord);
        X.push_back(local_current.x.value);
        Y.push_back(local_current.y.value);
        log(2) << "add local waypoint " << local_current << std::endl;
    }

    // define starting state
    FrenetCoordinate wp_last_frenet;
    if (count_previous >= 2) {
        log(2) << "create starting state from last 2 waypoints" << std::endl;
        auto wp_last = local_system.ToLocal(this->previous_path[count_previous - 1]);
        log(2) << "last waypoint " << wp_last << std::endl;
        wp_last_frenet = this->map.ConvertToFrenet(this->previous_path[count_previous - 1]);
    } else {
        log(2) << "create starting state from current position" << std::endl;
        auto wp_last = local_system.ToLocal(this->car.cartesian.coord);
        log(2) << "last waypoint " << wp_last << std::endl;
        wp_last_frenet = this->car.frenet;
    }

    // define end points
    FrenetCoordinate frenet_0_5_target(
        wp_last_frenet.s + 0.5 * this->map.GetFrenetSDistanceFromTo(wp_last_frenet.s, target_state.frenet.s),
        wp_last_frenet.d + 0.5 * (target_state.frenet.d - wp_last_frenet.d));
    auto cartesian_0_5_target = this->map.ConvertToCartesian(frenet_0_5_target);
    auto local_0_5_target = local_system.ToLocal(cartesian_0_5_target);
    X.push_back(local_0_5_target.x.value);
    Y.push_back(local_0_5_target.y.value);
    log(2) << "add local waypoint " << local_0_5_target << std::endl;

    auto local_target = local_system.ToLocal(target_state.cartesian.coord);
    X.push_back(local_target.x.value);
    Y.push_back(local_target.y.value);
    log(2) << "add local waypoint " << local_target << std::endl;
    auto total_x = local_target.x;

    FrenetCoordinate frenet_target_50m(
        target_state.frenet.s + 50_m,
        target_state.frenet.d);
    auto cartesian_target_50m = this->map.ConvertToCartesian(frenet_target_50m);
    auto local_target_50m = local_system.ToLocal(cartesian_target_50m);
    X.push_back(local_target_50m.x.value);
    Y.push_back(local_target_50m.y.value);
    log(2) << "add local waypoint " << local_target_50m << std::endl;

    FrenetCoordinate frenet_target_100m(
        target_state.frenet.s + 100_m,
        target_state.frenet.d);
    auto cartesian_target_100m = this->map.ConvertToCartesian(frenet_target_100m);
    auto local_target_100m = local_system.ToLocal(cartesian_target_100m);
    X.push_back(local_target_100m.x.value);
    Y.push_back(local_target_100m.y.value);
    log(2) << "add local waypoint " << local_target_100m << std::endl;

    // create spline
    tk::spline s;
    s.set_points(X, Y); // currently it is required that X is already sorted

    log(2) << std::endl;
    log(2) << "Calculate trajectory:" << std::endl;
    log(2) << "---------------------" << std::endl;
    // interpolate spline
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
            auto remaining_steps = count - i;
            auto delta_v = target_state.speed - last_speed;
            auto next_v = last_speed + delta_v / remaining_steps;
            auto dist_to_travel_next = next_v * timestep;
            auto angle_to_target = last.AngleTo(local_target);
            auto dist_in_x = dist_to_travel_next * cos(angle_to_target);
            auto next_x = last.x + dist_in_x;
            auto next_y = Distance(s(next_x.value));
            auto next = LocalCartesianCoordinate(next_x, next_y);
            auto dist = last.DistanceTo(next);
            auto speed = dist / timestep;
            trajectory.push_back(local_system.ToGlobal(next));
            last = next;
            last_speed = speed;
            log(2) << "add local waypoint " << next << " with speed " << speed << std::endl;
        }
    }

    return trajectory;
}

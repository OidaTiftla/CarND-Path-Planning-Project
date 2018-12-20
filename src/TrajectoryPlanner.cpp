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
        auto frenet = this->map.ConvertToFrenet(pos);
        auto speed = last2.DistanceTo(last) / timestep;
        start_state = VehicleState(-1, pos, frenet, speed);
        remaining_time_horizon = time_horizon - count_previous * timestep;
    }

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
        if (this->map.GetFrenetSDistanceFromTo(this->car.frenet.s, preceding_vehicle.frenet.s) > 0_m) {
            // preceding vehicle is currently in front of us

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
                log(1) << "too fast for vehicle in front of us" << std::endl;
                // set target speed to speed of vehicle in front of us
                target_speed = preceding_vehicle_prediction.speed;
                // and set the target s value to the s value of the preceding vehicle
                // minus the minimum distance to vehicle in front of us, if we drive with its speed
                auto min_distance_with_preceding_vehicle_speed = behavior.min_safety_zone_time * preceding_vehicle_prediction.speed;
                target_frenet.s = preceding_vehicle_prediction.frenet.s - min_distance_with_preceding_vehicle_speed;
                acceleration = (target_speed - start_state.speed) / remaining_time_horizon;
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
        log(2) << "reuse " << count_previous << " waypoints" << std::endl;
        for (int i = 0; i < count_previous; ++i) {
            auto local = local_system.ToLocal(this->previous_path[i]);
            X.push_back(local.x.value);
            Y.push_back(local.y.value);
            log(2) << "reuse local waypoint " << local << std::endl;
        }
    } else {
        log(2) << "create initial waypoints" << std::endl;
        auto local_current = local_system.ToLocal(this->car.cartesian.coord);
        X.push_back(local_current.x.value);
        Y.push_back(local_current.y.value);
        log(2) << "add local waypoint " << local_current << std::endl;

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
        GlobalCartesianCoordinate cartesian_0_1_time_horizon_after_current(
            this->car.cartesian.coord.x + (time_horizon * 0.1) * speed_x,
            this->car.cartesian.coord.y + (time_horizon * 0.1) * speed_y);
        auto local_0_1_time_horizon_after_current = local_system.ToLocal(cartesian_0_1_time_horizon_after_current);
        X.push_back(local_0_1_time_horizon_after_current.x.value);
        Y.push_back(local_0_1_time_horizon_after_current.y.value);
        log(2) << "add local waypoint " << local_0_1_time_horizon_after_current << std::endl;
    }

    // define end points
    FrenetCoordinate frenet_half_target(
        (car.frenet.s + target_state.frenet.s) / 2,
        (car.frenet.d + target_state.frenet.d) / 2);
    auto cartesian_half_target = this->map.ConvertToCartesian(frenet_half_target);
    auto local_half_target = local_system.ToLocal(cartesian_half_target);
    X.push_back(local_half_target.x.value);
    Y.push_back(local_half_target.y.value);
    log(2) << "add local waypoint " << local_half_target << std::endl;

    FrenetCoordinate frenet_0_8_target(
        car.frenet.s + 0.8 * (target_state.frenet.s - car.frenet.s),
        car.frenet.d + 0.8 * (target_state.frenet.d - car.frenet.d));
    auto cartesian_0_8_target = this->map.ConvertToCartesian(frenet_0_8_target);
    auto local_0_8_target = local_system.ToLocal(cartesian_0_8_target);
    X.push_back(local_0_8_target.x.value);
    Y.push_back(local_0_8_target.y.value);
    log(2) << "add local waypoint " << local_0_8_target << std::endl;

    FrenetCoordinate frenet_0_9_target(
        car.frenet.s + 0.9 * (target_state.frenet.s - car.frenet.s),
        car.frenet.d + 0.9 * (target_state.frenet.d - car.frenet.d));
    auto cartesian_0_9_target = this->map.ConvertToCartesian(frenet_0_9_target);
    auto local_0_9_target = local_system.ToLocal(cartesian_0_9_target);
    X.push_back(local_0_9_target.x.value);
    Y.push_back(local_0_9_target.y.value);
    log(2) << "add local waypoint " << local_0_9_target << std::endl;

    auto local_target = local_system.ToLocal(target_state.cartesian.coord);
    X.push_back(local_target.x.value);
    Y.push_back(local_target.y.value);
    log(2) << "add local waypoint " << local_target << std::endl;

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
            auto time_to_target = time_horizon - (i * timestep);
            auto acceleration = (target_state.speed - last_speed) / time_to_target;
            auto speed = last_speed + acceleration * timestep;
            auto dist_to_travel_next = speed * timestep;
            auto angle_to_target = last.AngleTo(local_target);
            auto dist_in_x = dist_to_travel_next * cos(angle_to_target);
            auto next_x = last.x + dist_in_x;
            auto next_y = Distance(s(next_x.value));
            auto next = LocalCartesianCoordinate(next_x, next_y);
            trajectory.push_back(local_system.ToGlobal(next));
            last = next;
            last_speed = speed;
            log(2) << "add local waypoint " << next << " with speed " << speed << " angle to target " << ToDegree(angle_to_target) << std::endl;
        }
    }

    return trajectory;
}

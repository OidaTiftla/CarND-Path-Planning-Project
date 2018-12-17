//
// Created by chmst on 10/11/2016.
//

#include "TrajectoryPlanner.h"


std::vector<GlobalCartesianCoordinate> TrajectoryPlanner::PlanNextTrajectory(const Behavior& behavior, const Time timestep, const Time time_horizon) const {
    FrenetCoordinate target_frenet(
        this->car.frenet.s + behavior.max_speed * time_horizon,
        this->map.GetFrenetDFromLane(behavior.lane));
    auto acceleration = (behavior.max_speed - this->car.speed) / time_horizon;
    if (acceleration > this->max_acceleration) {
        acceleration = this->max_acceleration;
    } else if (acceleration < -this->max_acceleration) {
        acceleration = -this->max_acceleration;
    }
    auto target_speed = this->car.speed + acceleration * time_horizon;

    auto target_vehicle_iter = std::find_if(this->sensor_fusion.begin(), this->sensor_fusion.end(), [&behavior] (const VehicleState& vehicle) { return behavior.vehicle_id == vehicle.id; } );
    if (target_vehicle_iter != this->sensor_fusion.end()) {
        auto target_vehicle = *target_vehicle_iter;
        auto target_vehicle_prediction = this->map.PredictIntoFuture(target_vehicle, time_horizon);
        auto min_distance_with_target_speed = behavior.min_distance_travel_time * target_speed;
        auto rel_s_target_vehicle_prediction = this->map.GetFrenetSDistanceFromTo(this->car.frenet.s, target_vehicle_prediction.frenet.s);
        auto rel_s_target = this->map.GetFrenetSDistanceFromTo(this->car.frenet.s, target_frenet.s);
        if (rel_s_target_vehicle_prediction - min_distance_with_target_speed < rel_s_target) {
            // to fast -> drive with speed of target vehicle
            target_speed = target_vehicle_prediction.speed;
            auto min_distance_with_target_vehicle_speed = behavior.min_distance_travel_time * target_vehicle_prediction.speed;
            target_frenet.s = target_vehicle_prediction.frenet.s - min_distance_with_target_vehicle_speed;
        }
    }

    auto target_cartesian = this->map.ConvertToCartesian(target_frenet);
    auto target_cartesian_1m = this->map.ConvertToCartesian(FrenetCoordinate(target_frenet.s + 1_m, target_frenet.d));
    auto target_theta = target_cartesian.AngleTo(target_cartesian_1m);
    VehicleState target_state(-1, GlobalCartesianPosition(target_cartesian.x, target_cartesian.y, target_theta), target_frenet, target_speed);

    // output target state
    log(1) << std::endl;
    log(1) << "Target state:" << std::endl;
    log(1) << "-------------" << std::endl;
    log(1) << "timestep: " << timestep << std::endl;
    log(1) << "time horizon: " << time_horizon << std::endl;
    log(1) << "s: " << target_state.frenet.s << std::endl;
    log(1) << "d: " << target_state.frenet.d << std::endl;
    log(1) << "speed: " << target_state.speed << std::endl;
    log(1) << "x: " << target_state.cartesian.coord.x << std::endl;
    log(1) << "y: " << target_state.cartesian.coord.y << std::endl;
    log(1) << "theta: " << ToDegree(target_state.cartesian.theta) << std::endl;
    log(1) << "speed x: " << target_state.speed_x << std::endl;
    log(1) << "speed y: " << target_state.speed_y << std::endl;

    return this->CalculateTrajectory(target_state, timestep, time_horizon);
}

std::vector<GlobalCartesianCoordinate> TrajectoryPlanner::CalculateTrajectory(const VehicleState& target_state, const Time timestep, const Time time_horizon) const {
    CoordinateSystemReference local_system(this->car.cartesian);

    log(2) << std::endl;
    log(2) << "Calculate spline:" << std::endl;
    log(2) << "-----------------" << std::endl;

    // spline control points
    std::vector<double> X, Y;

    // define starting points
    int count_previous = ceil(time_horizon * 0.2 / timestep);
    count_previous = std::min(count_previous, (int)this->previous_path.size());
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
    GlobalCartesianCoordinate cartesian_0_2_time_horizon_before_target(
        target_state.cartesian.coord.x - (time_horizon * 0.2) * target_state.speed_x,
        target_state.cartesian.coord.y - (time_horizon * 0.2) * target_state.speed_y);
    auto local_0_2_time_horizon_before_target = local_system.ToLocal(cartesian_0_2_time_horizon_before_target);
    X.push_back(local_0_2_time_horizon_before_target.x.value);
    Y.push_back(local_0_2_time_horizon_before_target.y.value);
    log(2) << "add local waypoint " << local_0_2_time_horizon_before_target << std::endl;

    GlobalCartesianCoordinate cartesian_0_1_time_horizon_before_target(
        target_state.cartesian.coord.x - (time_horizon * 0.1) * target_state.speed_x,
        target_state.cartesian.coord.y - (time_horizon * 0.1) * target_state.speed_y);
    auto local_0_1_time_horizon_before_target = local_system.ToLocal(cartesian_0_1_time_horizon_before_target);
    X.push_back(local_0_1_time_horizon_before_target.x.value);
    Y.push_back(local_0_1_time_horizon_before_target.y.value);
    log(2) << "add local waypoint " << local_0_1_time_horizon_before_target << std::endl;

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

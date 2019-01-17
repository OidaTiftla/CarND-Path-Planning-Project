#include "TrajectoryPlanner.h"
#include <algorithm>
#include "log.h"
#include "spline.h"
#if PLOTTRAJECTORY
#include "gnuplot-iostream.h"
#endif


// ToDo: avoid any collisions,
// because behavior planner is not responsible for this, he only makes a suggestion

std::vector<GlobalCartesianCoordinate> TrajectoryPlanner::plan_next_trajectory(const Behavior& behavior, const Time timestep, const Time time_horizon) const {
    auto logger = LogLevelStack(1);

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
        GlobalCartesianPosition pos(last.x, last.y, last2.angle_to(last));
        auto frenet = this->map.convert_to_frenet(last);
        auto speed = last2.distance_to(last) / timestep;
        start_state = VehicleState(-1, pos, frenet, speed);
        remaining_time_horizon = time_horizon - count_previous * timestep;
    }
    log(1) << "start: " << start_state.frenet << ", " << start_state.speed << ", " << start_state.cartesian << std::endl;
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
    auto target_distance = start_state.speed * remaining_time_horizon + 0.5 * acceleration * pow<2>(remaining_time_horizon);
    auto target_frenet = this->map.add_lane_distance(
        start_state.frenet,
        target_distance,
        this->map.get_frenet_d_from_lane(behavior.lane));
    auto target_cartesian = this->map.convert_to_cartesian_position(target_frenet);

    auto preceding_vehicle_iter = std::find_if(this->sensor_fusion.begin(), this->sensor_fusion.end(), [&behavior] (const VehicleState& vehicle) { return behavior.vehicle_id == vehicle.id; } );
    if (preceding_vehicle_iter != this->sensor_fusion.end()) {
        auto preceding_vehicle = *preceding_vehicle_iter;
        // preceding vehicle position, in the future
        auto preceding_vehicle_prediction = this->map.predict_into_future_onto_lane(preceding_vehicle, time_horizon);
        // minimum distance to vehicle in front of us, if we drive with desired target speed
        auto min_distance_with_target_speed = behavior.min_safety_zone_time * target_speed;
        // calculate s for both (the preceding vehicle in the future and me in the future if I drive with the desired target speed)
        // calculate s relative to my current car position (combats the fact, that s jumps when I drive over the starting line (s=0 wraparound))
        auto rel_dist_preceding_vehicle_prediction = this->map.get_lane_distance_from_to(this->car.frenet, preceding_vehicle_prediction.frenet);
        auto rel_dist_target = this->map.get_lane_distance_from_to(this->car.frenet, target_frenet);
        auto actual_distance_to_preceding_vehicle = rel_dist_preceding_vehicle_prediction - rel_dist_target;
        if (actual_distance_to_preceding_vehicle < min_distance_with_target_speed) {
            // to fast -> no safety zone -> drive with speed of preceding vehicle and with safety zone
            log(1) << "too fast for vehicle in front of us (s prediction: " << preceding_vehicle_prediction.frenet.s << ")" << std::endl;
            // calculate acceleration, needed to keep minimum distance to preceding vehicle
            acceleration = (rel_dist_preceding_vehicle_prediction - car.speed * (time_horizon + behavior.min_safety_zone_time))
                / (0.5 * pow<2>(time_horizon) + time_horizon * behavior.min_safety_zone_time);
            // validate acceleration
            if (acceleration > this->max_acceleration) {
                log(1) << "exceed max acceleration (preceding vehicle)" << std::endl;
                acceleration = this->max_acceleration;
            } else if (acceleration < -this->max_acceleration) {
                log(1) << "exceed max deceleration (preceding vehicle)" << std::endl;
                acceleration = -this->max_acceleration;
            }

            target_speed = start_state.speed + acceleration * remaining_time_horizon;
            if (target_speed < 0.1_m / 1_s) {
                target_speed = 0.1_m / 1_s;
                acceleration = (target_speed - start_state.speed) / remaining_time_horizon;
                log(1) << "exceed min speed (preceding vehicle) [new acceleration: " << acceleration << "]" << std::endl;
            }
            target_distance = start_state.speed * remaining_time_horizon + 0.5 * acceleration * pow<2>(remaining_time_horizon);
            target_frenet = this->map.add_lane_distance(
                start_state.frenet,
                target_distance,
                this->map.get_frenet_d_from_lane(behavior.lane));
            target_cartesian = this->map.convert_to_cartesian_position(target_frenet);
        }
    }

    VehicleState target_state(-1, target_cartesian, target_frenet, target_speed);

    // output target state
    log(1) << "s: " << target_state.frenet.s << std::endl;
    log(1) << "d: " << target_state.frenet.d << std::endl;
    log(1) << "speed: " << target_state.speed << std::endl;
    log(1) << "acceleration: " << (target_state.speed - start_state.speed) / remaining_time_horizon << std::endl;
    log(1) << "x: " << target_state.cartesian.coord.x << std::endl;
    log(1) << "y: " << target_state.cartesian.coord.y << std::endl;
    log(1) << "theta: " << to_degree(target_state.cartesian.theta) << std::endl;
    log(1) << "speed x: " << target_state.speed_x << std::endl;
    log(1) << "speed y: " << target_state.speed_y << std::endl;

    return this->calculate_trajectory(count_previous, target_state, timestep, time_horizon);
}

std::vector<GlobalCartesianCoordinate> TrajectoryPlanner::calculate_trajectory(const int count_previous, const VehicleState& target_state, const Time timestep, const Time time_horizon) const {
    CoordinateSystemReference local_system(this->car.cartesian);

    log(2) << std::endl;
    log(2) << "Calculate spline:" << std::endl;
    log(2) << "-----------------" << std::endl;

    // spline control points
    std::vector<double> X, Y;

    // define starting points
    if (count_previous >= 2) {
        log(2) << "reuse " << count_previous << " waypoints" << std::endl;
        for (int i = count_previous - 2; i < count_previous; ++i) {
            auto local = local_system.to_local(this->previous_path[i]);
            X.push_back(local.x.value);
            Y.push_back(local.y.value);
            log(2) << "reuse local waypoint " << local << std::endl;
        }
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
        auto local_minus_0_1_target = local_system.to_local(cartesian_minus_0_1_target);
        X.push_back(local_minus_0_1_target.x.value);
        Y.push_back(local_minus_0_1_target.y.value);
        log(2) << "add local waypoint " << local_minus_0_1_target << std::endl;

        auto local_current = local_system.to_local(this->car.cartesian.coord);
        X.push_back(local_current.x.value);
        Y.push_back(local_current.y.value);
        log(2) << "add local waypoint " << local_current << std::endl;
    }

    // define starting state
    FrenetCoordinate wp_last_frenet;
    if (count_previous >= 2) {
        log(2) << "create starting state from last 2 waypoints" << std::endl;
        auto wp_last = local_system.to_local(this->previous_path[count_previous - 1]);
        wp_last_frenet = this->map.convert_to_frenet(this->previous_path[count_previous - 1]);
        log(2) << "last waypoint " << wp_last << ", " << wp_last_frenet << std::endl;
    } else {
        log(2) << "create starting state from current position" << std::endl;
        auto wp_last = local_system.to_local(this->car.cartesian.coord);
        wp_last_frenet = this->car.frenet;
        log(2) << "last waypoint " << wp_last << ", " << wp_last_frenet << std::endl;
    }

    // define end points
    auto local_target = local_system.to_local(target_state.cartesian.coord);
    // X.push_back(local_target.x.value);
    // Y.push_back(local_target.y.value);
    // log(2) << "add local waypoint " << local_target << std::endl;

    FrenetCoordinate frenet_30m(
        this->car.frenet.s + 30_m,
        target_state.frenet.d);
    auto cartesian_30m = this->map.convert_to_cartesian(frenet_30m);
    auto local_30m = local_system.to_local(cartesian_30m);
    X.push_back(local_30m.x.value);
    Y.push_back(local_30m.y.value);
    log(2) << "add local waypoint " << local_30m << std::endl;

    FrenetCoordinate frenet_60m(
        this->car.frenet.s + 60_m,
        target_state.frenet.d);
    auto cartesian_60m = this->map.convert_to_cartesian(frenet_60m);
    auto local_60m = local_system.to_local(cartesian_60m);
    X.push_back(local_60m.x.value);
    Y.push_back(local_60m.y.value);
    log(2) << "add local waypoint " << local_60m << std::endl;

    FrenetCoordinate frenet_90m(
        this->car.frenet.s + 90_m,
        target_state.frenet.d);
    auto cartesian_90m = this->map.convert_to_cartesian(frenet_90m);
    auto local_90m = local_system.to_local(cartesian_90m);
    X.push_back(local_90m.x.value);
    Y.push_back(local_90m.y.value);
    log(2) << "add local waypoint " << local_90m << std::endl;

    // create spline
    tk::spline s;
    s.set_points(X, Y); // currently it is required that X is already sorted

    log(2) << std::endl;
    log(2) << "Calculate trajectory:" << std::endl;
    log(2) << "---------------------" << std::endl;
    // interpolate spline
    std::vector<GlobalCartesianCoordinate> trajectory;
    int count = ceil(time_horizon / timestep);
    auto last = local_system.to_local(this->car.cartesian.coord);
    auto last_speed = this->car.speed;
    log(2) << "current car " << last << " with speed " << last_speed << std::endl;
    log(2) << "create " << count << " waypoints" << std::endl;
    for (int i = 0; i < count; ++i) {
        if (i < count_previous) {
            auto next = local_system.to_local(this->previous_path[i]);
            auto dist = last.distance_to(next);
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
            auto angle_to_target = last.angle_to(local_target);
            auto dist_in_x = dist_to_travel_next * cos(angle_to_target);
            auto next_x = last.x + dist_in_x;
            auto next_y = Distance(s(next_x.value));
            auto next = LocalCartesianCoordinate(next_x, next_y);
            auto dist = last.distance_to(next);
            auto speed = dist / timestep;
            trajectory.push_back(local_system.to_global(next));
            last = next;
            last_speed = speed;
            log(2) << "add local waypoint " << next << " with speed " << speed << std::endl;
        }
    }

#if PLOTTRAJECTORY
    static Gnuplot gp;

    auto steps = 20;
    auto distance = 100_m;
    auto t_step = time_horizon / steps;
    // gp << "set xrange [" << -2 << ":" << (bPlanner.max_lanes * map.lane_width + 2_m).value << "]\n";
    gp << "set xrange [-" << distance.value << ":" << distance.value << "]\n";
    gp << "set yrange [-" << distance.value << ":" << distance.value << "]\n";
    gp << "set size ratio -1\n";
    gp << "plot";
    bool first = true;
    for (int lane = 0; lane <= this->map.max_lanes; ++lane) {
        if (first) {
            first = false;
        } else {
            gp << ",";
        }
        gp << " '-' with lines title 'lane " << lane << "'";
    }
    gp << ", '-' with points title 'vehicle ego'";
    gp << ", '-' with lines title 'spline'";
    gp << "\n";

    std::vector<std::pair<double, double>> points;
    // draw lanes
    for (int lane = 0; lane <= this->map.max_lanes; ++lane) {
        points.clear();
        auto d = this->map.get_frenet_d_from_lane(lane) - (this->map.lane_width / 2);
        for (auto s_diff = -distance; s_diff <= distance; s_diff += distance / steps) {
            auto s = car.frenet.s + s_diff;
            auto coord = this->map.convert_to_cartesian(FrenetCoordinate(s, d));
            auto local = local_system.to_local(coord);
            points.push_back(std::make_pair(-local.y.value, local.x.value));
        }
        gp.send1d(points);
    }
    // draw ego trajectory
    points.clear();
    for (auto i = 0; i < steps; ++i) {
        auto local = local_system.to_local(trajectory[i * trajectory.size() / steps]);
        points.push_back(std::make_pair(-local.y.value, local.x.value));
    }
    gp.send1d(points);
    // draw spline
    points.clear();
    for (auto x = -distance; x <= distance; x += distance / steps) {
        LocalCartesianCoordinate local(x, Distance(s(x.value)));
        points.push_back(std::make_pair(-local.y.value, local.x.value));
    }
    gp.send1d(points);
    gp.flush();
#endif

    return trajectory;
}

#include "Map.h"
#include <cmath>
#include "spline.h"


size_t Map::closest_way_point_index(const GlobalCartesianCoordinate pos) const {
    Distance closestLen = 1000000_m; // large number
    size_t closest_way_point_index = 0;

    for (size_t i = 0; i < this->wayPoints.size(); i++) {
        auto dist = this->wayPoints[i].cartesian.distance_to(pos);
        if (dist < closestLen) {
            closestLen = dist;
            closest_way_point_index = i;
        }
    }
    return closest_way_point_index;
}

size_t Map::next_way_point_index(const GlobalCartesianCoordinate pos) const {
    auto closest_way_point_index = this->closest_way_point_index(pos);

    auto angleToRightStart = LocalCartesianCoordinate(0_m, 0_m).angle_to(this->wayPoints[closest_way_point_index].normalized_normal_vector);
    auto angleToPos = this->wayPoints[closest_way_point_index].cartesian.angle_to(pos);
    auto angleFromRightStartToPos = normalize_around_zero(angleToPos - angleToRightStart);

    if (angleFromRightStartToPos >= AngleRad(0)) {
        closest_way_point_index++;
        if (closest_way_point_index >= this->wayPoints.size()) {
            closest_way_point_index = 0;
        }
    }

    return closest_way_point_index;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
FrenetCoordinate Map::convert_to_frenet(const GlobalCartesianCoordinate pos) const {
    auto next_wp = this->next_way_point_index(pos);
    auto prev_wp = (this->wayPoints.size() + next_wp - 1) % this->wayPoints.size();

    auto n = this->wayPoints[next_wp].cartesian - this->wayPoints[prev_wp].cartesian;
    auto x = pos - this->wayPoints[prev_wp].cartesian;

    // find the projection of x onto n
    auto proj_norm = (x.x * n.x + x.y * n.y) / (n.x * n.x + n.y * n.y);
    LocalCartesianCoordinate proj(proj_norm * n.x, proj_norm * n.y);

    auto frenet_d = x.distance_to(proj);

    // see if d value is positive or negative by looking at the angles

    auto angleToNext = this->wayPoints[prev_wp].cartesian.angle_to(this->wayPoints[next_wp].cartesian);
    auto angleToPos = this->wayPoints[prev_wp].cartesian.angle_to(pos);
    auto angleFromNextToPos = normalize_around_zero(angleToPos - angleToNext);

    if (angleFromNextToPos >= AngleRad(0)) {
        frenet_d *= -1;
    }

    // calculate s value
    auto frenet_s = 0_m;
    for (size_t i = 0; i < prev_wp; i++) {
        frenet_s += this->wayPoints[i].cartesian.distance_to(this->wayPoints[i + 1].cartesian);
    }

    frenet_s += LocalCartesianCoordinate(0_m, 0_m).distance_to(proj);

    // create spline
    CoordinateSystemReference local_system(this->wayPoints[prev_wp].cartesian.x, this->wayPoints[prev_wp].cartesian.y, angleToNext);
    std::vector<double> X, Y;
    for (int i = -2; i < 4; ++i) {
        size_t wp = (this->wayPoints.size() + prev_wp + i) % this->wayPoints.size();
        auto local = local_system.to_local(this->wayPoints[wp].cartesian);
        X.push_back(local.x.value);
        Y.push_back(local.y.value);
    }
    tk::spline s;
    s.set_points(X, Y); // currently it is required that X is already sorted

    // the s along the segment
    auto seg_s = frenet_s - this->wayPoints[prev_wp].frenet.s;

    auto d_curvature_correction = Distance(s(seg_s.value));

    return { frenet_s, frenet_d + d_curvature_correction };
}

// Transform from Frenet s,d coordinates to Cartesian x,y
GlobalCartesianCoordinate Map::convert_to_cartesian(FrenetCoordinate pos) const {
    return this->convert_to_cartesian_position(pos).coord;
}

GlobalCartesianPosition Map::convert_to_cartesian_position(FrenetCoordinate pos) const {
    // normalize input for s: 0 < s < max_s
    pos.s = this->normalize_s(pos.s);

    size_t prev_wp = 0;

    while (pos.s > this->wayPoints[prev_wp + 1].frenet.s && (prev_wp < (this->wayPoints.size() - 1))) {
        prev_wp++;
    }

    size_t next_wp = (prev_wp + 1) % this->wayPoints.size();

    AngleRad heading = this->wayPoints[prev_wp].cartesian.angle_to(this->wayPoints[next_wp].cartesian);

    // create spline
    CoordinateSystemReference local_system(this->wayPoints[prev_wp].cartesian.x, this->wayPoints[prev_wp].cartesian.y, heading);
    std::vector<double> X, Y;
    for (int i = -2; i < 4; ++i) {
        size_t wp = (this->wayPoints.size() + prev_wp + i) % this->wayPoints.size();
        auto local = local_system.to_local(this->wayPoints[wp].cartesian);
        X.push_back(local.x.value);
        Y.push_back(local.y.value);
    }
    tk::spline s;
    s.set_points(X, Y); // currently it is required that X is already sorted

    // the s along the segment
    auto seg_s = pos.s - this->wayPoints[prev_wp].frenet.s;

    auto d_curvature_correction = Distance(s(seg_s.value));
    auto angle_correction = atan2(Distance(s((seg_s + 0.05_m).value)) - Distance(s((seg_s - 0.05_m).value)), 0.1_m);

    LocalCartesianPosition pos_local(seg_s, -pos.d + d_curvature_correction, angle_correction);

    return local_system.to_global(pos_local);
}

int Map::get_lane_from(const FrenetCoordinate frenet) const {
    return floor(frenet.d / this->lane_width);
}

Distance Map::get_frenet_d_from_lane(const int lane) const {
    return lane * this->lane_width + (0.5 * this->lane_width);
}

Distance Map::get_frenet_s_distance_from_to(const Distance from, const Distance to) const {
    auto dist = to - from;
    return this->normalize_s(dist);
}

Distance Map::get_lane_distance_from_to(const FrenetCoordinate from, const FrenetCoordinate to) const {
    auto s_diff = this->get_frenet_s_distance_from_to(from.s, to.s);
    auto steps = ceil(s_diff / 5_m);
    auto s_step = s_diff / steps;
    auto d_step = (to.d - from.d) / steps;

    auto distance = 0_m;
    auto last = this->convert_to_cartesian(from);
    for (int i = 1; i <= steps; ++i) {
        FrenetCoordinate next_frenet(s_step * i + from.s, d_step * i + from.d);
        auto next = this->convert_to_cartesian(next_frenet);
        distance += last.distance_to(next);
        last = next;
    }
    return distance;
}

FrenetCoordinate Map::add_lane_distance(const FrenetCoordinate from, const Distance distance, const Distance target_d) const {
    auto s_step = distance / 5;
    auto s = from.s + distance;

    FrenetCoordinate to(s, target_d);
    auto min_diff = abs(this->get_lane_distance_from_to(from, to) - distance);
    while (s_step > 0.2_m) {
        FrenetCoordinate to(s + s_step, target_d);
        auto diff = abs(this->get_lane_distance_from_to(from, to) - distance);
        if (diff < min_diff) {
            min_diff = diff;
            s += s_step;
        } else {
            FrenetCoordinate to(s - s_step, target_d);
            auto diff = abs(this->get_lane_distance_from_to(from, to) - distance);
            if (diff < min_diff) {
                min_diff = diff;
                s -= s_step;
            } else {
                s_step /= 2;
            }
        }
    }
    return FrenetCoordinate(s, target_d);
}

FrenetCoordinate Map::add_s_distance(const FrenetCoordinate from, const Distance distance, const Distance target_d) const {
    return FrenetCoordinate(from.s + distance, target_d);
}

Distance Map::normalize_s(Distance s) const {
    while (s >= this->max_s) {
        s -= this->max_s;
    }
    while (s < 0_m) {
        s += this->max_s;
    }
    return s;
}

VehicleState Map::predict_into_future_in_cartesian(const VehicleState& vehicle, const Time time_horizon) const {
    // constant speed and angle (x and y speed)
    GlobalCartesianPosition future_pos(
        vehicle.cartesian.coord.x + vehicle.speed_x * time_horizon,
        vehicle.cartesian.coord.y + vehicle.speed_y * time_horizon,
        vehicle.cartesian.theta);
    return VehicleState(
        vehicle.id,
        future_pos,
        this->convert_to_frenet(future_pos.coord),
        vehicle.speed);
}

VehicleState Map::predict_into_future_in_frenet(const VehicleState& vehicle, const Time time_horizon) const {
    // constant speed in s and d
    CoordinateSystemReference local_system(this->convert_to_cartesian_position(vehicle.frenet));
    auto coord_1_second_future = vehicle.cartesian.coord + LocalCartesianCoordinate(vehicle.speed_x * 1_s, vehicle.speed_y * 1_s);
    auto local_x_y_dist_per_second = local_system.to_local(coord_1_second_future) - local_system.to_local(vehicle.cartesian.coord);
    auto s_speed = local_x_y_dist_per_second.x / 1_s;
    auto d_speed = -local_x_y_dist_per_second.y / 1_s; // y with minus because d is in the opposite direction of y
    auto current_frenet = this->convert_to_frenet(vehicle.cartesian.coord);
    auto theta_frenet = vehicle.cartesian.theta - this->convert_to_cartesian_position(current_frenet).theta;
    FrenetCoordinate future_pos_frenet(
        current_frenet.s + s_speed * time_horizon,
        current_frenet.d + d_speed * time_horizon);
    auto future_pos = this->convert_to_cartesian_position(future_pos_frenet);
    future_pos.theta += theta_frenet;
    return VehicleState(
        vehicle.id,
        future_pos,
        future_pos_frenet,
        vehicle.speed);
}

VehicleState Map::predict_into_future_onto_lane(const VehicleState& vehicle, const Time time_horizon) const {
    // constant speed in s and d
    auto s_speed = vehicle.speed;
    auto d_speed = 0_m / 1_s;
    auto current_frenet = this->convert_to_frenet(vehicle.cartesian.coord);
    auto theta_frenet = vehicle.cartesian.theta - this->convert_to_cartesian_position(current_frenet).theta;
    FrenetCoordinate future_pos_frenet(
        current_frenet.s + s_speed * time_horizon,
        current_frenet.d + d_speed * time_horizon);
    auto future_pos = this->convert_to_cartesian_position(future_pos_frenet);
    future_pos.theta += theta_frenet;
    return VehicleState(
        vehicle.id,
        future_pos,
        future_pos_frenet,
        vehicle.speed);
}

VehicleState Map::predict_into_future_combined(const VehicleState& vehicle, const Time time_horizon) const {
    // constant speed in s and d
    CoordinateSystemReference local_system(this->convert_to_cartesian_position(vehicle.frenet));
    auto coord_1_second_future = vehicle.cartesian.coord + LocalCartesianCoordinate(vehicle.speed_x * 1_s, vehicle.speed_y * 1_s);
    auto local_x_y_dist_per_second = local_system.to_local(coord_1_second_future) - local_system.to_local(vehicle.cartesian.coord);
    auto s_speed = local_x_y_dist_per_second.x / 1_s;
    auto d_speed = -local_x_y_dist_per_second.y / 1_s; // y with minus because d is in the opposite direction of y
    if (d_speed < 0.5_m / 1_s) {
        d_speed = 0_m / 1_s;
    }
    auto current_frenet = this->convert_to_frenet(vehicle.cartesian.coord);
    auto theta_frenet = vehicle.cartesian.theta - this->convert_to_cartesian_position(current_frenet).theta;
    FrenetCoordinate future_pos_frenet(
        current_frenet.s + s_speed * time_horizon,
        current_frenet.d + d_speed * time_horizon);
    auto future_pos = this->convert_to_cartesian_position(future_pos_frenet);
    future_pos.theta += theta_frenet;
    return VehicleState(
        vehicle.id,
        future_pos,
        future_pos_frenet,
        vehicle.speed);
}

int Map::find_next_vehicle_in_lane(const Distance start_s, const int lane, const std::vector<VehicleState> &sensor_fusion) const {
    // find nearest vehicle in same lane in front of us
    // set initial value to the maximum search distance
    auto vehicle_id = -1;
    auto nearest = 999999_m;
    for (auto vehicle : sensor_fusion) {
        if (this->get_lane_from(vehicle.frenet) == lane) {
            auto dist = this->get_frenet_s_distance_from_to(start_s, vehicle.frenet.s);
            if (dist < nearest) {
                nearest = dist;
                vehicle_id = vehicle.id;
            }
        }
    }
    return vehicle_id;
}

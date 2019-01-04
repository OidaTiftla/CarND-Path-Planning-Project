#include "Map.h"


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

size_t Map::next_way_point_index(const GlobalCartesianPosition pos) const {
    size_t closest_way_point_index = this->closest_way_point_index(pos.coord);

    AngleRad heading = pos.angle_to(this->wayPoints[closest_way_point_index].cartesian);

    AngleRad angle(abs(heading));
    angle = AngleRad(std::min(AngleRad(2 * M_PI) - angle, angle));

    if (angle > AngleRad(M_PI / 4)) {
        closest_way_point_index++;
        if (closest_way_point_index >= this->wayPoints.size()) {
            closest_way_point_index = 0;
        }
    }

    return closest_way_point_index;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
FrenetCoordinate Map::convert_to_frenet(const GlobalCartesianCoordinate pos) const {
    auto wp = this->closest_way_point_index(pos);
    auto angle = LocalCartesianCoordinate(0_m, 0_m).angle_to(this->wayPoints[wp].normalized_normal_vector) + to_radian(90_deg);

    auto next_wp = this->next_way_point_index(GlobalCartesianPosition(pos.x, pos.y, angle));

    auto prev_wp = (next_wp - 1) % this->wayPoints.size();

    auto n = this->wayPoints[next_wp].cartesian - this->wayPoints[prev_wp].cartesian;
    auto x = pos - this->wayPoints[prev_wp].cartesian;

    // find the projection of x onto n
    auto proj_norm = (x.x * n.x + x.y * n.y) / (n.x * n.x + n.y * n.y);
    LocalCartesianCoordinate proj(proj_norm * n.x, proj_norm * n.y);

    auto frenet_d = x.distance_to(proj);

    // see if d value is positive or negative by comparing it to a center point

    auto center = GlobalCartesianCoordinate(1000_m, 2000_m) - this->wayPoints[prev_wp].cartesian;
    auto centerToPos = center.distance_to(x);
    auto centerToRef = center.distance_to(proj);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    auto frenet_s = 0_m;
    for (size_t i = 0; i < prev_wp; i++) {
        frenet_s += this->wayPoints[i].cartesian.distance_to(this->wayPoints[i + 1].cartesian);
    }

    frenet_s += LocalCartesianCoordinate(0_m, 0_m).distance_to(proj);

    return { frenet_s, frenet_d };
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

    size_t wp2 = (prev_wp + 1) % this->wayPoints.size();

    AngleRad heading = this->wayPoints[prev_wp].cartesian.angle_to(this->wayPoints[wp2].cartesian);
    // the x,y,s along the segment
    auto seg_s = pos.s - this->wayPoints[prev_wp].frenet.s;

    auto seg_x = this->wayPoints[prev_wp].cartesian.x + seg_s * cos(heading);
    auto seg_y = this->wayPoints[prev_wp].cartesian.y + seg_s * sin(heading);

    auto perp_heading = heading - AngleRad(M_PI / 2);

    auto x = seg_x + pos.d * cos(perp_heading);
    auto y = seg_y + pos.d * sin(perp_heading);

    return { x, y, heading };
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

Distance Map::normalize_s(Distance s) const {
    while (s >= this->max_s) {
        s -= this->max_s;
    }
    while (s < 0_m) {
        s += this->max_s;
    }
    return s;
}

VehicleState Map::predict_into_future(const VehicleState& vehicle, const Time time_horizon) const {
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

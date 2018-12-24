//
// Created by chmst on 10/11/2016.
//

#include "Map.h"


int Map::ClosestWayPointIndex(const GlobalCartesianCoordinate pos) const {
    Distance closestLen = 1000000_m; // large number
    int closestWayPointIndex = 0;

    for (int i = 0; i < this->wayPoints.size(); i++) {
        auto dist = this->wayPoints[i].cartesian.DistanceTo(pos);
        if (dist < closestLen) {
            closestLen = dist;
            closestWayPointIndex = i;
        }
    }
    return closestWayPointIndex;
}

int Map::NextWayPointIndex(const GlobalCartesianPosition pos) const {
    int closestWayPointIndex = this->ClosestWayPointIndex(pos.coord);

    AngleRad heading = pos.AngleTo(this->wayPoints[closestWayPointIndex].cartesian);

    AngleRad angle(abs(heading));
    angle = AngleRad(std::min(AngleRad(2 * M_PI) - angle, angle));

    if (angle > AngleRad(M_PI / 4)) {
        closestWayPointIndex++;
        if (closestWayPointIndex >= this->wayPoints.size()) {
            closestWayPointIndex = 0;
        }
    }

    return closestWayPointIndex;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
FrenetCoordinate Map::ConvertToFrenet(const GlobalCartesianCoordinate pos) const {
    int wp = this->ClosestWayPointIndex(pos);
    auto angle = LocalCartesianCoordinate(0_m, 0_m).AngleTo(this->wayPoints[wp].normalized_normal_vector) + ToRadian(90_deg);

    int next_wp = this->NextWayPointIndex(GlobalCartesianPosition(pos.x, pos.y, angle));

    int prev_wp;
    prev_wp = (next_wp - 1) % this->wayPoints.size();

    auto n = this->wayPoints[next_wp].cartesian - this->wayPoints[prev_wp].cartesian;
    auto x = pos - this->wayPoints[prev_wp].cartesian;

    // find the projection of x onto n
    auto proj_norm = (x.x * n.x + x.y * n.y) / (n.x * n.x + n.y * n.y);
    LocalCartesianCoordinate proj(proj_norm * n.x, proj_norm * n.y);

    auto frenet_d = x.DistanceTo(proj);

    // see if d value is positive or negative by comparing it to a center point

    auto center = GlobalCartesianCoordinate(1000_m, 2000_m) - this->wayPoints[prev_wp].cartesian;
    auto centerToPos = center.DistanceTo(x);
    auto centerToRef = center.DistanceTo(proj);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    auto frenet_s = 0_m;
    for (int i = 0; i < prev_wp; i++) {
        frenet_s += this->wayPoints[i].cartesian.DistanceTo(this->wayPoints[i + 1].cartesian);
    }

    frenet_s += LocalCartesianCoordinate(0_m, 0_m).DistanceTo(proj);

    return { frenet_s, frenet_d };
}

// Transform from Frenet s,d coordinates to Cartesian x,y
GlobalCartesianCoordinate Map::ConvertToCartesian(FrenetCoordinate pos) const {
    // normalize input for s: 0 < s < max_s
    pos.s = this->NormalizeS(pos.s);

    int prev_wp = -1;

    while (pos.s > this->wayPoints[prev_wp + 1].frenet.s && (prev_wp < (int)(this->wayPoints.size() - 1))) {
        prev_wp++;
    }

    int wp2 = (prev_wp + 1) % this->wayPoints.size();

    AngleRad heading = this->wayPoints[prev_wp].cartesian.AngleTo(this->wayPoints[wp2].cartesian);
    // the x,y,s along the segment
    auto seg_s = pos.s - this->wayPoints[prev_wp].frenet.s;

    auto seg_x = this->wayPoints[prev_wp].cartesian.x + seg_s * cos(heading);
    auto seg_y = this->wayPoints[prev_wp].cartesian.y + seg_s * sin(heading);

    auto perp_heading = heading - AngleRad(M_PI / 2);

    auto x = seg_x + pos.d * cos(perp_heading);
    auto y = seg_y + pos.d * sin(perp_heading);

    return { x, y };
}

int Map::GetLaneFrom(const FrenetCoordinate frenet) const {
    return floor(frenet.d / this->lane_width);
}

Distance Map::GetFrenetDFromLane(const int lane) const {
    return lane * this->lane_width + (0.5 * this->lane_width);
}

Distance Map::GetFrenetSDistanceFromTo(const Distance from, const Distance to) const {
    auto dist = to - from;
    return this->NormalizeS(dist);
}

Distance Map::NormalizeS(Distance s) const {
    while (s >= this->max_s) {
        s -= this->max_s;
    }
    while (s < 0_m) {
        s += this->max_s;
    }
    return s;
}

VehicleState Map::PredictIntoFuture(const VehicleState& vehicle, const Time time_horizon) const {
    GlobalCartesianPosition future_pos(
        vehicle.cartesian.coord.x + vehicle.speed_x * time_horizon,
        vehicle.cartesian.coord.y + vehicle.speed_y * time_horizon,
        vehicle.cartesian.theta);
    return VehicleState(
        vehicle.id,
        future_pos,
        this->ConvertToFrenet(future_pos.coord),
        vehicle.speed);
}

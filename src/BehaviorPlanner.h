#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>
#include <algorithm>
#include "log.h"
#include "SemanticTypes.h"
#include "Map.h"
#include "TrajectoryCost.h"
#include "VehicleState.hpp"
#include "TrajectoryKinematics.hpp"


struct Behavior {
    int vehicle_id = -1;
    Speed max_speed = 0_m / 1_s;
    Time min_safety_zone_time = 0_s;
    int lane = -1;
};

enum class BehaviorState {
    ConstantSpeed,
    KeepLane,
    PrepareLaneChangeLeft,
    PrepareLaneChangeRight,
    LaneChangeLeft,
    LaneChangeRight,
};

inline std::ostream& operator<<(std::ostream& out, const BehaviorState& o) {
    switch (o) {
        case BehaviorState::ConstantSpeed:
            return out << "ConstantSpeed";
        case BehaviorState::KeepLane:
            return out << "KeepLane";
        case BehaviorState::PrepareLaneChangeLeft:
            return out << "PrepareLaneChangeLeft";
        case BehaviorState::PrepareLaneChangeRight:
            return out << "PrepareLaneChangeRight";
        case BehaviorState::LaneChangeLeft:
            return out << "LaneChangeLeft";
        case BehaviorState::LaneChangeRight:
            return out << "LaneChangeRight";
        default:
            return out << "Unknown";
    };
}

class BehaviorPlanner {
    BehaviorState state = BehaviorState::ConstantSpeed;
    int lane = -1000; // initialize when first car variable is passed into the planner
    TrajectoryCost cost;

public:
    Map map;
    Speed max_speed;
    Time min_safety_zone_time;
    Time lane_change_min_safety_zone_time;
    Acceleration max_acceleration;
    Jerk max_jerk;
    int max_lanes;
    Distance vehicle_length;
    Distance vehicle_width;

    BehaviorPlanner(
        const Map &map,
        const Speed max_speed,
        const Time min_safety_zone_time,
        const Acceleration max_acceleration,
        const Jerk max_jerk,
        const int max_lanes,
        const Distance vehicle_length,
        const Distance vehicle_width)
        : map(map),
          max_speed(max_speed),
          min_safety_zone_time(min_safety_zone_time),
          lane_change_min_safety_zone_time(0.5 * min_safety_zone_time),
          max_acceleration(max_acceleration),
          max_jerk(max_jerk),
          max_lanes(max_lanes),
          vehicle_length(vehicle_length),
          vehicle_width(vehicle_width),
          cost(map, max_speed, min_safety_zone_time, max_acceleration, max_jerk, vehicle_length, vehicle_width) {}

    Behavior plan_next_behavior(
        const VehicleState &car,
        const Time timestep,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion);

    TrajectoryKinematics try_follow_vehicle(
        const VehicleState &car,
        const int vehicle_id,
        const int target_lane,
        const int intended_lane,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    std::vector<BehaviorState> successor_states() const;

    TrajectoryKinematics generate_trajectory(
        const BehaviorState state,
        const VehicleState &car,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    TrajectoryKinematics constant_speed_trajectory(
        const VehicleState &car,
        const Time time_horizon) const;

    TrajectoryKinematics keep_lane_trajectory(
        const VehicleState &car,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    TrajectoryKinematics prep_lane_change_trajectory(
        const BehaviorState state,
        const VehicleState &car,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    TrajectoryKinematics lane_change_trajectory(
        const BehaviorState state,
        const VehicleState &car,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;
};

#endif //BEHAVIOR_PLANNER_H

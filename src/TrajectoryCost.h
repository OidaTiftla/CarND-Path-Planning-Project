#ifndef TRAJECTORY_COST_H
#define TRAJECTORY_COST_H

#include <vector>
#include <algorithm>
#include <functional>
#include <cmath>
#include "log.h"
#include "SemanticTypes.h"
#include "Map.h"
#include "VehicleState.hpp"
#include "TrajectoryKinematics.hpp"


class TrajectoryCost {
public:
    Map map;
    Speed max_speed;
    Time min_safety_zone_time;
    Acceleration max_acceleration;
    Jerk max_jerk;
    Distance vehicle_length;
    Distance vehicle_width;

    TrajectoryCost(
        const Map &map,
        const Speed max_speed,
        const Time min_safety_zone_time,
        const Acceleration max_acceleration,
        const Jerk max_jerk,
        const Distance vehicle_length,
        const Distance vehicle_width)
        : map(map),
          max_speed(max_speed),
          min_safety_zone_time(min_safety_zone_time),
          max_acceleration(max_acceleration),
          max_jerk(max_jerk),
          vehicle_length(vehicle_length),
          vehicle_width(vehicle_width) {}

    float calculate_cost(
        const TrajectoryKinematics &trajectory,
        const VehicleState &car,
        const Time timestep,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion);

    VehicleState evaluate_trajectory(
        const TrajectoryKinematics &trajectory,
        const Time t) const;

    float buffer_cost(
        const TrajectoryKinematics &trajectory,
        const VehicleState &car,
        const Time timestep,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    float collision_cost(
        const TrajectoryKinematics &trajectory,
        const VehicleState &car,
        const Time timestep,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    float safety_zone_cost(
        const TrajectoryKinematics &trajectory,
        const VehicleState &car,
        const Time timestep,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    float stays_on_road_cost(
        const TrajectoryKinematics &trajectory,
        const VehicleState &car,
        const Time timestep,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    float exceeds_speed_limit_cost(
        const TrajectoryKinematics &trajectory,
        const VehicleState &car,
        const Time timestep,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    float efficiency_cost(
        const TrajectoryKinematics &trajectory,
        const VehicleState &car,
        const Time timestep,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    float total_acceleration_cost(
        const TrajectoryKinematics &trajectory,
        const VehicleState &car,
        const Time timestep,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    float max_acceleration_cost(
        const TrajectoryKinematics &trajectory,
        const VehicleState &car,
        const Time timestep,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    float total_jerk_cost(
        const TrajectoryKinematics &trajectory,
        const VehicleState &car,
        const Time timestep,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    float max_jerk_cost(
        const TrajectoryKinematics &trajectory,
        const VehicleState &car,
        const Time timestep,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;

    float inefficiency_cost(
        const TrajectoryKinematics &trajectory,
        const VehicleState &car,
        const Time timestep,
        const Time time_horizon,
        const std::vector<VehicleState> &sensor_fusion) const;
};

#endif //TRAJECTORY_COST_H

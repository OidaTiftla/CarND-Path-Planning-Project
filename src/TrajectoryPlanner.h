#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <vector>
#include "SemanticTypes.h"
#include "Map.h"
#include "VehicleState.hpp"
#include "BehaviorPlanner.h"


class TrajectoryPlanner {
public:
    Map map;
    Acceleration max_acceleration;
    Jerk max_jerk;
    VehicleState car;
    std::vector<GlobalCartesianCoordinate> previous_path;
    FrenetCoordinate end_path;
    std::vector<VehicleState> sensor_fusion;

    TrajectoryPlanner(const Map& map, const Acceleration max_acceleration, const Jerk max_jerk, const VehicleState& car, const std::vector<GlobalCartesianCoordinate>& previous_path, FrenetCoordinate& end_path, const std::vector<VehicleState>& sensor_fusion) : map(map), max_acceleration(max_acceleration), max_jerk(max_jerk), car(car), previous_path(previous_path), end_path(end_path), sensor_fusion(sensor_fusion) {}

    std::vector<GlobalCartesianCoordinate> plan_next_trajectory(const Behavior& behavior, const Time timestep, const Time time_horizon) const;
    std::vector<GlobalCartesianCoordinate> calculate_trajectory(const int count_previous, const VehicleState& target_state, const Time timestep, const Time time_horizon) const;
};

#endif //TRAJECTORY_PLANNER_H

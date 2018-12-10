//
// Created by chmst on 10/11/2016.
//

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
    VehicleState car;
    std::vector<GlobalCartesianCoordinate> previous_path;
    FrenetCoordinate end_path;

    TrajectoryPlanner(const Map& map, const VehicleState& car, const std::vector<GlobalCartesianCoordinate>& previous_path, FrenetCoordinate& end_path/*, const Prediction& sensor_fusion*/) : map(map), car(car), previous_path(previous_path), end_path(end_path) {}

    std::vector<GlobalCartesianCoordinate> PlanNextTrajectory(const Behavior& behavior, const Time timestep, const Time time_horizon) const;
};

#endif //TRAJECTORY_PLANNER_H

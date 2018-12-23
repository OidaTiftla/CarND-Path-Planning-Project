//
// Created by chmst on 10/11/2016.
//

#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include "log.h"
#include "SemanticTypes.h"
#include "Map.h"
#include "VehicleState.hpp"


struct Behavior {
    int vehicle_id = -1;
    Speed max_speed = 0_m / 1_s;
    Time min_safety_zone_time = 0_s;
    int lane = -1;
};

class BehaviorPlanner {
public:
    Map map;
    Speed max_speed;
    Time min_safety_zone_time;
    Acceleration max_acceleration;
    Jerk max_jerk;

    BehaviorPlanner(const Map& map, const Speed max_speed, const Time min_safety_zone_time, const Acceleration max_acceleration, const Jerk max_jerk) : map(map), max_speed(max_speed), min_safety_zone_time(min_safety_zone_time), max_acceleration(max_acceleration), max_jerk(max_jerk) {}

    Behavior PlanNextBehavior(const VehicleState& car, const std::vector<VehicleState>& sensor_fusion, const Time timestep, const Time time_horizon) const;
};

#endif //BEHAVIOR_PLANNER_H

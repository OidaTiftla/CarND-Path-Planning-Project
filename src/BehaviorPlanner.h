//
// Created by chmst on 10/11/2016.
//

#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include "SemanticTypes.h"
#include "Map.h"
#include "VehicleState.hpp"


struct Behavior {
    int vehicle_id = -1;
    Speed max_speed = 0_m / 1_s;
    Distance min_distance = 0_m;
    int lane = -1;
};

class BehaviorPlanner {
public:
    Map map;
    Speed max_speed;
    Time min_distance_travel_time;

    BehaviorPlanner(const Map& map, const Speed max_speed, const Time min_distance_travel_time) : map(map), max_speed(max_speed), min_distance_travel_time(min_distance_travel_time) {}

    Behavior PlanNextBehavior(const VehicleState& car, const std::vector<VehicleState>& sensor_fusion) const;
};

#endif //BEHAVIOR_PLANNER_H

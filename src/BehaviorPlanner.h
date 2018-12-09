//
// Created by chmst on 10/11/2016.
//

#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include "SemanticTypes.h"
#include "Map.h"
#include "Vehicle.hpp"


struct Behavior {
    int vehicle_id = -1;
    Speed max_speed = 0_m / 1_s;
    Distance min_distance = 0_m;
    int lane = -1;
};

class BehaviorPlanner {
public:
    Map map;

    BehaviorPlanner(const Map& map) : map(map) {}

    Behavior PlanNextBehavior(const Vehicle& car/*, const Prediction& sensor_fusion*/) const;
};

#endif //BEHAVIOR_PLANNER_H

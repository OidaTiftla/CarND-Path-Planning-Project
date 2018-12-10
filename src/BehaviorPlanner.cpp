//
// Created by chmst on 10/11/2016.
//

#include "BehaviorPlanner.h"


Behavior BehaviorPlanner::PlanNextBehavior(const VehicleState& car, const std::vector<VehicleState>& sensor_fusion) const {
    Behavior behavior;
    behavior.lane = this->map.GetLaneFrom(car.frenet);
    behavior.max_speed = this->max_speed;
    behavior.min_distance = this->min_distance_travel_time * this->max_speed;
    behavior.vehicle_id = -1;
    return behavior;
}

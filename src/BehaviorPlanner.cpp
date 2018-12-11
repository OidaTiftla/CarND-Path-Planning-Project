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
    // find nearest vehicle in same lane in front of us
    auto nearest = this->map.max_s;
    for (auto vehicle : sensor_fusion) {
        if (this->map.GetLaneFrom(vehicle.frenet) == behavior.lane) {
            auto dist = vehicle.frenet.s - car.frenet.s;
            if (dist <= 0_m) {
                dist += this->map.max_s;
            }
            if (dist < nearest) {
                nearest = dist;
                behavior.vehicle_id = vehicle.id;
            }
        }
    }
    return behavior;
}

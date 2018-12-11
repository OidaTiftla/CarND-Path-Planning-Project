//
// Created by chmst on 10/11/2016.
//

#include "BehaviorPlanner.h"


Behavior BehaviorPlanner::PlanNextBehavior(const VehicleState& car, const std::vector<VehicleState>& sensor_fusion) const {
    Behavior behavior;
    behavior.lane = this->map.GetLaneFrom(car.frenet);
    behavior.max_speed = this->max_speed;
    behavior.min_distance_travel_time = this->min_distance_travel_time;
    behavior.vehicle_id = -1;
    // find nearest vehicle in same lane in front of us
    auto nearest = this->map.max_s;
    for (auto vehicle : sensor_fusion) {
        if (this->map.GetLaneFrom(vehicle.frenet) == behavior.lane) {
            auto dist = this->map.GetFrenetSDistanceFromTo(car.frenet.s, vehicle.frenet.s);
            if (dist < nearest) {
                nearest = dist;
                behavior.vehicle_id = vehicle.id;
            }
        }
    }
    return behavior;
}

//
// Created by chmst on 10/11/2016.
//

#include "BehaviorPlanner.h"


Behavior BehaviorPlanner::PlanNextBehavior(const VehicleState& car, const std::vector<VehicleState>& sensor_fusion) const {
    log(1) << std::endl;
    log(1) << "Sensor fusion:" << std::endl;
    log(1) << "--------------" << std::endl;

    Behavior behavior;
    behavior.lane = this->map.GetLaneFrom(car.frenet);
    behavior.max_speed = this->max_speed;
    behavior.min_safety_zone_time = this->min_safety_zone_time;
    behavior.vehicle_id = -1;
    // find nearest vehicle in same lane in front of us
    auto nearest = this->map.max_s;
    for (auto vehicle : sensor_fusion) {
        log(1) << "distance to " << vehicle.id << " (" << vehicle.cartesian << ", " << vehicle.speed << "): " << vehicle.cartesian.DistanceTo(car.cartesian) << std::endl;
        if (this->map.GetLaneFrom(vehicle.frenet) == behavior.lane) {
            auto dist = this->map.GetFrenetSDistanceFromTo(car.frenet.s, vehicle.frenet.s);
            if (dist < nearest) {
                nearest = dist;
                behavior.vehicle_id = vehicle.id;
            }
        }
    }
    log(1) << "nearest: " << nearest << " (id: " << behavior.vehicle_id << ")" << std::endl;
    return behavior;
}

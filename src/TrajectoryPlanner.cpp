//
// Created by chmst on 10/11/2016.
//

#include "TrajectoryPlanner.h"


std::vector<GlobalCartesianCoordinate> TrajectoryPlanner::PlanNextTrajectory(const Behavior& behavior, const Time timestep, const Time time_horizon) const {
    FrenetCoordinate target_frenet(
        this->car.frenet.s + behavior.max_speed * time_horizon,
        this->map.GetFrenetDFromLane(behavior.lane));
    auto target_speed = behavior.max_speed;

    auto target_vehicle_iter = std::find_if(this->sensor_fusion.begin(), this->sensor_fusion.end(), [&behavior] (const VehicleState& vehicle) { return behavior.vehicle_id == vehicle.id; } );
    if (target_vehicle_iter != this->sensor_fusion.end()) {
        auto target_vehicle = *target_vehicle_iter;
        auto target_vehicle_prediction = this->map.PredictIntoFuture(target_vehicle, time_horizon);
        auto min_distance_with_target_speed = behavior.min_distance_travel_time * target_speed;
        auto rel_s_target_vehicle_prediction = this->map.GetFrenetSDistanceFromTo(this->car.frenet.s, target_vehicle_prediction.frenet.s);
        auto rel_s_target = this->map.GetFrenetSDistanceFromTo(this->car.frenet.s, target_frenet.s);
        if (rel_s_target_vehicle_prediction < rel_s_target) {
            // to fast -> drive with speed of target vehicle
            target_speed = target_vehicle_prediction.speed;
            auto min_distance_with_target_vehicle_speed = behavior.min_distance_travel_time * target_vehicle_prediction.speed;
            target_frenet.s = target_vehicle_prediction.frenet.s - min_distance_with_target_vehicle_speed;
        }
    }

    auto target_cartesian = this->map.ConvertToCartesian(target_frenet);

    // ToDo: Spline
    std::vector<GlobalCartesianCoordinate> trajectory;
    GlobalCartesianCoordinate last = this->car.cartesian.coord;
    int count = ceil(time_horizon / timestep);
    for (int i = 0; i < count; ++i) {
        if (i < this->previous_path.size()) {
            last = this->previous_path[i];
            trajectory.push_back(last);
        } else {
            auto speed = 20_mph;
            last = last + LocalCartesianCoordinate(speed * timestep, 0_m);
            trajectory.push_back(last);
        }
    }
    return trajectory;
}

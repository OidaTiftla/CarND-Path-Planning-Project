//
// Created by chmst on 10/11/2016.
//

#include "TrajectoryPlanner.h"


std::vector<GlobalCartesianCoordinate> TrajectoryPlanner::PlanNextTrajectory(const Behavior& behavior, const Time timestep, const Time time_horizon) const {
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

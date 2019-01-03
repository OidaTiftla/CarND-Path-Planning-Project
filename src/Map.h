#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <cmath>
#include <vector>
#include "WayPoint.hpp"
#include "SemanticTypes.h"
#include "VehicleState.hpp"


class Map {
public:
    std::vector<WayPoint> wayPoints;
    Distance max_s;
    Distance lane_width;

    Map() : wayPoints(), max_s(0), lane_width(0) {}
    Map(std::vector<WayPoint> wayPoints, Distance max_s, Distance lane_width) : wayPoints(wayPoints), max_s(max_s), lane_width(lane_width) {}

    size_t ClosestWayPointIndex(const GlobalCartesianCoordinate pos) const;
    size_t NextWayPointIndex(const GlobalCartesianPosition pos) const;

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    FrenetCoordinate ConvertToFrenet(const GlobalCartesianCoordinate pos) const;

    // Transform from Frenet s,d coordinates to Cartesian x,y
    GlobalCartesianCoordinate ConvertToCartesian(FrenetCoordinate pos) const;
    GlobalCartesianPosition ConvertToCartesianPosition(FrenetCoordinate pos) const;

    int GetLaneFrom(const FrenetCoordinate frenet) const;
    Distance GetFrenetDFromLane(const int lane) const;
    Distance GetFrenetSDistanceFromTo(const Distance from, const Distance to) const;
    Distance NormalizeS(Distance s) const;

    VehicleState PredictIntoFuture(const VehicleState& vehicle, const Time time_horizon) const;

    int find_next_vehicle_in_lane(const Distance start_s, const int lane, const std::vector<VehicleState> &sensor_fusion) const;
};

#endif //MAP_H

#ifndef MAP_H
#define MAP_H

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

    size_t closest_way_point_index(const GlobalCartesianCoordinate pos) const;
    size_t next_way_point_index(const GlobalCartesianPosition pos) const;

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    FrenetCoordinate convert_to_frenet(const GlobalCartesianCoordinate pos) const;

    // Transform from Frenet s,d coordinates to Cartesian x,y
    GlobalCartesianCoordinate convert_to_cartesian(FrenetCoordinate pos) const;
    GlobalCartesianPosition convert_to_cartesian_position(FrenetCoordinate pos) const;

    int get_lane_from(const FrenetCoordinate frenet) const;
    Distance get_frenet_d_from_lane(const int lane) const;
    Distance get_frenet_s_distance_from_to(const Distance from, const Distance to) const;
    Distance get_lane_distance_from_to(const FrenetCoordinate from, const FrenetCoordinate to) const;
    FrenetCoordinate add_lane_distance(const FrenetCoordinate from, const Distance distance, const Distance target_d) const;
    Distance normalize_s(Distance s) const;

    VehicleState predict_into_future(const VehicleState& vehicle, const Time time_horizon) const;

    int find_next_vehicle_in_lane(const Distance start_s, const int lane, const std::vector<VehicleState> &sensor_fusion) const;
};

#endif //MAP_H

#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "log.h"
#include "SemanticTypes.h"
#include "Map.h"
#include "VehicleState.hpp"
#include "BehaviorPlanner.h"
#include "TrajectoryPlanner.h"
#if PLOTMAP | ANALYZEMAPONLY
#include <algorithm>
#include "gnuplot-iostream.h"
#endif

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<WayPoint> map_waypoints;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    auto max_s = 6945.554_m;
    auto lane_width = 4_m;
    auto max_speed = 49_mph;
    auto min_safety_zone_time = 1.0_s;
    auto max_acceleration = 0.5 * 10_m / 1_s / 1_s; //10_m / 1_s / 1_s;
    auto max_jerk = 0.5 * 10_m / 1_s / 1_s / 1_s; //10_m / 1_s / 1_s / 1_s;
    auto timestep = 0.02_s;
    auto time_horizon = 2.0_s;
    auto max_lanes = 3;
    auto vehicle_length = 4.5_m;
    auto vehicle_width = 3_m;

    auto logger = LogLevelStack(1, "main: ");

    // output config
    log(0) << endl;
    log(0) << "Configuration settings:" << endl;
    log(0) << "-----------------------" << endl;
    log(0) << "max s: " << max_s << endl;
    log(0) << "lane width: " << lane_width << endl;
    log(0) << "max speed: " << max_speed << endl;
    log(0) << "min safety zone time: " << min_safety_zone_time << endl;
    log(0) << "max acceleration: " << max_acceleration << endl;
    log(0) << "max jerk: " << max_jerk << endl;
    log(0) << "timestep: " << timestep << endl;
    log(0) << "time horizon: " << time_horizon << endl;
    log(0) << "max lanes: " << max_lanes << endl;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints.push_back(
            WayPoint(
                GlobalCartesianCoordinate(Distance(x), Distance(y)),
                FrenetCoordinate(Distance(s), 0_m),
                LocalCartesianCoordinate(Distance(d_x), Distance(d_y))));
    }

    Map map(map_waypoints, max_s, lane_width, max_lanes);

#if ANALYZEMAPONLY

    Gnuplot gp_map;
    gp_map << "set size ratio -1\n";
    gp_map << "plot '-' with linespoints title 'waypoints'";
    for (int lane = 0; lane <= max_lanes; ++lane) {
        gp_map << ", '-' with linespoints title 'lane " << lane << "'";
    }
    gp_map << "\n";
    std::vector<std::pair<double, double>> points;
    // draw waypoints
    for (auto wp : map.wayPoints) {
        points.push_back(std::make_pair(wp.cartesian.x.value, wp.cartesian.y.value));
    }
    gp_map.send1d(points);
    // draw lanes
    for (int lane = 0; lane <= max_lanes; ++lane) {
        points.clear();
        auto d = map.get_frenet_d_from_lane(lane) - (map.lane_width / 2);
        for (auto s = 0_m; s <= max_s; s += 2_m) {
            auto coord = map.convert_to_cartesian(FrenetCoordinate(s, d));
            points.push_back(std::make_pair(coord.x.value, coord.y.value));
            coord = map.convert_to_cartesian(map.convert_to_frenet(coord));
            points.push_back(std::make_pair(coord.x.value, coord.y.value));
        }
        gp_map.send1d(points);
    }
    gp_map.flush();

    while (true);

#else

    BehaviorPlanner bPlanner(map,
        max_speed,
        min_safety_zone_time,
        max_acceleration,
        max_jerk,
        max_lanes,
        vehicle_length,
        vehicle_width);

#if PLOTSIGNALS
    auto start_time = std::chrono::system_clock::now();
#endif

#if PLOTMAP
    Gnuplot gp;
#endif

    h.onMessage([
#if PLOTSIGNALS
        &start_time,
#endif
#if PLOTMAP
        &gp,
#endif
        &map, &bPlanner, &timestep, &time_horizon, &max_acceleration, &max_jerk](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
        uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    GlobalCartesianPosition car_cartesian(j[1]["x"] * 1_m, j[1]["y"] * 1_m, to_radian(j[1]["yaw"] * 1_deg));
                    FrenetCoordinate car_frenet(j[1]["s"] * 1_m, j[1]["d"] * 1_m);
                    auto car_speed = j[1]["speed"] * 1_mph;
                    VehicleState car(-1, car_cartesian, map.convert_to_frenet(car_cartesian.coord), car_speed);

                    // Previous path data given to the Planner
                    vector<GlobalCartesianCoordinate> previous_path;
                    for (size_t i = 0; i < j[1]["previous_path_x"].size(); ++i) {
                        previous_path.push_back(GlobalCartesianCoordinate(j[1]["previous_path_x"][i] * 1_m, j[1]["previous_path_y"][i] * 1_m));
                    }
                    // Previous path's end s and d values
                    FrenetCoordinate end_path(j[1]["end_path_s"] * 1_m, j[1]["end_path_d"] * 1_m);

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    vector<VehicleState> sensor_fusion;
                    for (auto vehicle_fusion : j[1]["sensor_fusion"]) {
                        int id = vehicle_fusion[0];
                        auto x = vehicle_fusion[1] * 1_m;
                        auto y = vehicle_fusion[2] * 1_m;
                        auto vx = vehicle_fusion[3] * 1_m / 1_s;
                        auto vy = vehicle_fusion[4] * 1_m / 1_s;
                        auto s = vehicle_fusion[5] * 1_m;
                        auto d = vehicle_fusion[6] * 1_m;
                        GlobalCartesianCoordinate cartesian(x, y);
                        FrenetCoordinate frenet(s, d);
                        sensor_fusion.push_back(VehicleState(id, cartesian, map.convert_to_frenet(cartesian), vx, vy));
                    }


                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
#if PLOTSIGNALS
                    auto now = std::chrono::system_clock::now();
                    std::chrono::duration<double> timespan_since_start = now - start_time;
                    log_set_time(timespan_since_start.count());
#endif

#if PLOTMAP
                    CoordinateSystemReference local_system(car.cartesian);
                    auto steps = 20;
                    auto distance = 100_m;
                    auto t_step = time_horizon / steps;
                	// gp << "set xrange [" << -2 << ":" << (bPlanner.max_lanes * map.lane_width + 2_m).value << "]\n";
                    gp << "set xrange [-" << distance.value << ":" << distance.value << "]\n";
                    gp << "set yrange [-" << distance.value << ":" << distance.value << "]\n";
                    gp << "set size ratio -1\n";
                    gp << "plot";
                    bool first = true;
                    for (int lane = 0; lane <= bPlanner.max_lanes; ++lane) {
                        if (first) {
                            first = false;
                        } else {
                            gp << ",";
                        }
                        gp << " '-' with lines title 'lane " << lane << "'";
                    }
                    for (auto it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
                        if (first) {
                            first = false;
                        } else {
                            gp << ",";
                        }
                        gp << " '-' with lines title 'vehicle " << it->id << "'";
                    }
                    if (first) {
                        first = false;
                    } else {
                        gp << ",";
                    }
                    gp << " '-' with points title 'vehicle ego'";
                    gp << "\n";

                    // draw lanes
                    for (int lane = 0; lane <= bPlanner.max_lanes; ++lane) {
                        std::vector<std::pair<double, double>> points;
                        auto d = map.get_frenet_d_from_lane(lane) - (map.lane_width / 2);
                        for (auto s_diff = -distance; s_diff <= distance; s_diff += distance / steps) {
                            auto s = car.frenet.s + s_diff;
                            auto coord = map.convert_to_cartesian(FrenetCoordinate(s, d));
                            auto local = local_system.to_local(coord);
                            points.push_back(std::make_pair(-local.y.value, local.x.value));
                        }
                        gp.send1d(points);
                    }
                    // draw predictions
                    for (auto it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
                        std::vector<std::pair<double, double>> points;
                        for (auto t = 0_s; t <= time_horizon + 0.001_s; t += t_step) {
                            auto prediction = map.predict_into_future_onto_lane(*it, t);
                            auto local = local_system.to_local(prediction.cartesian.coord);
                            points.push_back(std::make_pair(-local.y.value, local.x.value));
                        }
                        gp.send1d(points);
                    }
#endif

                    // output car
                    log(3) << endl;
                    log(3) << "Car:" << endl;
                    log(3) << "----" << endl;
                    log(3) << "s: " << car.frenet.s << endl;
                    log(3) << "d: " << car.frenet.d << endl;
                    log(3) << "speed: " << car.speed << endl;
                    log(3) << "x: " << car.cartesian.coord.x << endl;
                    log(3) << "y: " << car.cartesian.coord.y << endl;
                    log(3) << "theta: " << to_degree(car.cartesian.theta) << endl;
                    log(3) << "speed x: " << car.speed_x << endl;
                    log(3) << "speed y: " << car.speed_y << endl;

#if PLOTSIGNALS
                    log_signal("car speed", car.speed.value);
                    log_signal("d", ((car.frenet.d - (map.lane_width / 2)) / map.lane_width).value);
#endif

                    auto behavior = bPlanner.plan_next_behavior(car, timestep, time_horizon, sensor_fusion);

                    TrajectoryPlanner tPlanner(map, max_acceleration, max_jerk, car, previous_path, end_path, sensor_fusion);
                    auto trajectory = tPlanner.plan_next_trajectory(behavior, timestep, time_horizon);

#if PLOTMAP
                    std::vector<std::pair<double, double>> points;
                    for (auto i = 0; i < steps; ++i) {
                        auto local = local_system.to_local(trajectory[i * trajectory.size() / steps]);
                        points.push_back(std::make_pair(-local.y.value, local.x.value));
                    }
                    gp.send1d(points);
                    gp.flush();
#endif
#if PLOTSIGNALS
                    log_signal("speed limit", (50_mph).value);
                    plot_signals();
#endif


                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    for (auto pos : trajectory) {
                        next_x_vals.push_back(pos.x.value);
                        next_y_vals.push_back(pos.y.value);
                        // cout << pos << endl;
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
        size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([
#if PLOTSIGNALS
        &start_time,
#endif
        &h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
#if PLOTSIGNALS
        start_time = std::chrono::system_clock::now();
#endif
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
        char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();

#endif
}
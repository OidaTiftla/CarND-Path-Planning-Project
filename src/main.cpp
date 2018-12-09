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
#include "SemanticTypes.h"
#include "Map.h"

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

    Map map(map_waypoints, max_s);

    h.onMessage([&map](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
                    GlobalCartesianPosition car_cartesian(j[1]["x"] * 1_m, j[1]["y"] * 1_m, j[1]["yaw"] * 1_rad);
                    FrenetCoordinate car_frenet(j[1]["s"] * 1_m, j[1]["d"] * 1_m);
                    auto car_speed = j[1]["speed"] * 1_m / 1_s;

                    // Previous path data given to the Planner
                    vector<GlobalCartesianCoordinate> previous_path;
                    for (int i = 0; i < j[1]["previous_path_x"].size(); ++i) {
                        previous_path.push_back(GlobalCartesianCoordinate(j[1]["previous_path_x"][i] * 1_m, j[1]["previous_path_y"][i] * 1_m));
                    }
                    // Previous path's end s and d values
                    FrenetCoordinate end_path(j[1]["end_path_s"] * 1_m, j[1]["end_path_d"] * 1_m);

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];


                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    vector<GlobalCartesianCoordinate> next_path;
                    GlobalCartesianCoordinate last = car_cartesian.coord;
                    for (int i = 0; i < 50; ++i) {
                        if (i < previous_path.size()) {
                            last = previous_path[i];
                            next_path.push_back(last);
                        } else {
                            auto speed = 5_m / 1_s;
                            auto delta_t = 0.02_s;
                            last = last + LocalCartesianCoordinate(speed * delta_t, 0_m);
                            next_path.push_back(last);
                        }
                    }


                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    for (auto pos : next_path) {
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

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
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
}
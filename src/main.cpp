#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <utility>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "Trajectory.h"

#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::array;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // push points to spline for later smooting lane
  tk::spline waypoint_spline_x;
  tk::spline waypoint_spline_y;
  tk::spline waypoint_spline_dx;
  tk::spline waypoint_spline_dy;

  std::vector<double> prev_s;
  std::vector<double> prev_d;

  waypoint_spline_x.set_points(map_waypoints_s, map_waypoints_x);
  waypoint_spline_y.set_points(map_waypoints_s, map_waypoints_y);
  waypoint_spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
  waypoint_spline_dy.set_points(map_waypoints_s, map_waypoints_dy);

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &waypoint_spline_x,
               &waypoint_spline_y, &waypoint_spline_dx, &waypoint_spline_dy, &prev_s, &prev_d]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = Helpers::hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = Helpers::milesPerHourToMetersPerSecond(j[1]["speed"]);

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          //auto sensor_fusion = j[1]["sensor_fusion"];
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          vector<double> next_x;
          vector<double> next_y;

          // first attempt: let the car drive in lane while maintaining speed
          static int lane = 1;

          if(previous_path_x.size() < 10) {

            double s;
            double d;

            if(previous_path_x.size() > 0) {

              for(size_t i = 0; i < previous_path_x.size(); ++i) {
                next_x.push_back(previous_path_x.at(i));
                next_y.push_back(previous_path_y.at(i));
              }

              prev_s.erase(prev_s.begin(), prev_s.begin() + prev_s.size() - previous_path_x.size());
              prev_d.erase(prev_d.begin(), prev_d.begin() + prev_d.size() - previous_path_y.size());

              s = prev_s.back();
              d = prev_d.back();
            }
            else {
              prev_s.clear();
              prev_d.clear();
              s = car_s;
              d = car_d;
            }

            // increase speed by 1m/s/s
            double target_speed = std::min(20., car_speed + 5);
            Trajectory::JMT jmt({s, car_speed, 0}, {s + 2 * target_speed, target_speed, 0}, 2);
            Trajectory::JMT jmt2({d, 0, 0}, {(2. + lane * 4.), 0, 0}, 2);

            for(int i = 1; i < (50 - previous_path_x.size()); ++i) {
              double s = jmt.get(i * 0.02);
              double d = jmt2.get(i * 0.02);

              //vector<double> xy = Helpers::getXY(s, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              prev_s.push_back(s);
              prev_d.push_back(d);
              vector<double> xy = Helpers::getXY(s, d, waypoint_spline_x, waypoint_spline_y, waypoint_spline_dx, waypoint_spline_dy);

              next_x.push_back(xy.at(0));
              next_y.push_back(xy.at(1));
            }

          } else {
            for(size_t i = 0; i < previous_path_x.size(); ++i) {
              next_x.push_back(previous_path_x.at(i));
              next_y.push_back(previous_path_y.at(i));
            }
          }

          msgJson["next_x"] = next_x;
          msgJson["next_y"] = next_y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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

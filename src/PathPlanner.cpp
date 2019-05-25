#include "Parameter.h"
#include "PathPlanner.h"
#include "helpers.h"

PathPlanner::PathPlanner(const std::vector<double>& waypoints_s,
  const std::vector<double>& waypoints_x,
  const std::vector<double> waypoints_y,
  const std::vector<double> waypoints_dx,
  const std::vector<double> waypoints_dy)
{
  // push points to spline for later smooting lane
  waypoint_spline_x.set_points(waypoints_s, waypoints_x);
  waypoint_spline_y.set_points(waypoints_s, waypoints_y);
  waypoint_spline_dx.set_points(waypoints_s, waypoints_dx);
  waypoint_spline_dy.set_points(waypoints_s, waypoints_dy);
}

std::vector<std::vector<double>> PathPlanner::update(const Car& ego,
    const std::vector<double>& previous_path_x,
    const std::vector<double>& previous_path_y,
    const std::vector<Car>& other_cars)
{
  // output vectors
  std::vector<double> next_x;
  std::vector<double> next_y;

  // check current lane from state and set if needed
  if(current_state.lane < 0) {
    current_state.lane = ego.getLane();
  }

  // update the previous s and d values according to the points not driven by
  // the simulator in cycle
  previous_path_s.erase(previous_path_s.begin(), previous_path_s.begin()
    + previous_path_s.size() - previous_path_x.size());

  previous_path_d.erase(previous_path_d.begin(), previous_path_d.begin()
    + previous_path_d.size() - previous_path_y.size());

  // first attempt: let the car drive in lane while maintaining speed
  if(previous_path_x.size() < 10) {

    double s;
    double d;

    // reuse previous path and use last s/d as base for new trajectory calculation
    if(previous_path_x.size() > 0) {

      for(size_t i = 0; i < previous_path_x.size(); ++i) {
        next_x.push_back(previous_path_x.at(i));
        next_y.push_back(previous_path_y.at(i));
      }

      s = previous_path_s.back();
      d = previous_path_d.back();
    }
    else {
      previous_path_s.clear();
      previous_path_d.clear();
      s = ego.getS();
      d = ego.getD();
    }

    auto future = behavior_handler.plan(Car(ego.getId(), s, d, ego.getSpeed()), other_cars);

    auto trajectory = trajectory_handler.GenerateTrajectory({s, ego.getSpeed(), 0},
      {d, 0, 0}, {future.getS(), future.getSpeed(), 0}, {future.getD(), 0, 0},
      Parameter::k_prediction_time);


    for(int i = 1; i < (50 - previous_path_x.size()); ++i) {
      double s = TrajectoryHandler::getJmtPos(trajectory.c_s, i * 0.02);
      double d = TrajectoryHandler::getJmtPos(trajectory.c_d, i * 0.02);

      //vector<double> xy = Helpers::getXY(s, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      previous_path_s.push_back(s);
      previous_path_d.push_back(d);
      std::vector<double> xy = Helpers::getXY(s, d, waypoint_spline_x, waypoint_spline_y, waypoint_spline_dx, waypoint_spline_dy);

      next_x.push_back(xy.at(0));
      next_y.push_back(xy.at(1));
    }

  } else {
    for(size_t i = 0; i < previous_path_x.size(); ++i) {
      next_x.push_back(previous_path_x.at(i));
      next_y.push_back(previous_path_y.at(i));
    }
  }

  return {next_x, next_y};
}
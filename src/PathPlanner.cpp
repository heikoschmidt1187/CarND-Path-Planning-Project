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

int PathPlanner::getLane(double d)
{
  for(unsigned i = 0; i < PathPlanner::k_lane_count; ++i) {
    if(d < (PathPlanner::k_lane_width * (i + 1))) {
      return i;
    }
  }
}

std::vector<std::vector<double>> PathPlanner::update(const Car& ego,
    const std::vector<double>& previous_path_x,
    const std::vector<double>& previous_path_y)
{
  // output vectors
  std::vector<double> next_x;
  std::vector<double> next_y;

  // check current lane from state and set if needed
  if(current_state.lane < 0) {
    current_state.lane = getLane(ego.getD());
  }


  // first attempt: let the car drive in lane while maintaining speed
  if(previous_path_x.size() < 10) {
    // update the previous s and d values according to the points not driven by
    // the simulator in cycle
    previous_path_s.erase(previous_path_s.begin(), previous_path_s.begin()
      + previous_path_s.size() - previous_path_x.size());

    previous_path_d.erase(previous_path_d.begin(), previous_path_d.begin()
      + previous_path_d.size() - previous_path_y.size());

    double s;
    double d;

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

    // increase speed by 1m/s/s
    double target_speed = std::min(static_cast<double>(k_speed_limit - k_speed_buffer),
      ego.getSpeed() + 5);

    Trajectory::JMT jmt({s, ego.getSpeed(), 0}, {s + 2 * target_speed, target_speed, 0}, 2);
    Trajectory::JMT jmt2({d, 0, 0}, {(2. + current_state.lane * 4.), 0, 0}, 2);

    for(int i = 1; i < (50 - previous_path_x.size()); ++i) {
      double s = jmt.get(i * 0.02);
      double d = jmt2.get(i * 0.02);

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

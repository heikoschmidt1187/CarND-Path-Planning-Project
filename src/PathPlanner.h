#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <vector>

#include "spline.h"
#include "Trajectory.h"
#include "Car.h"

class PathPlanner {
public:
  PathPlanner(const std::vector<double>& waypoints_s,
    const std::vector<double>& waypoints_x,
    const std::vector<double> waypoints_y,
    const std::vector<double> waypoints_dx,
    const std::vector<double> waypoints_dy);

  std::vector<std::vector<double>> update(const Car& ego,
    const std::vector<double>& previous_path_x,
    const std::vector<double>& previous_path_y);

private:
  static constexpr float k_lane_width = 4.0;        // m
  static constexpr unsigned int k_lane_count = 3;   // number of lanes

  static constexpr float k_speed_limit = 22.2;      // m/s
  static constexpr float k_speed_buffer = 2.;       // m/s

private:
  struct PlannerState {
    PlannerState()
      : lane(-1)
    {}

    int lane;
  };

private:
  int getLane(double d);

private:
  std::vector<double> previous_path_s;
  std::vector<double> previous_path_d;

  tk::spline waypoint_spline_x;
  tk::spline waypoint_spline_y;
  tk::spline waypoint_spline_dx;
  tk::spline waypoint_spline_dy;

  PlannerState current_state;
};

#endif /* PATHPLANNER_H_ */

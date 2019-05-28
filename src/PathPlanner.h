#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <vector>

#include "spline.h"

#include "Parameter.h"
#include "BehaviorHandler.h"
#include "TrajectoryHandler.h"
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
    const std::vector<double>& previous_path_y,
    const std::vector<Car>& other_cars,
    const std::vector<double>& map_waypoints_s,
    const std::vector<double>& map_waypoints_x,
    const std::vector<double>& map_waypoints_y);

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
  std::vector<Car::State> previous_path_s;
  std::vector<Car::State> previous_path_d;

  tk::spline waypoint_spline_x;
  tk::spline waypoint_spline_y;
  tk::spline waypoint_spline_dx;
  tk::spline waypoint_spline_dy;

  PlannerState current_state;

  TrajectoryHandler trajectory_handler;
  BehaviorHandler behavior_handler;
};

#endif /* PATHPLANNER_H_ */

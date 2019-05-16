#include <utility>

#include "PathPlanner.h"

array<vector<double>, 2> PathPlanner::plan(const array<double, 6>& localizationData,
      const vector<double>& prev_path_x, const vector<double>& prev_path_y,
      const array<double, 2>& end_path_Frenet,
      const vector<vector<double>>& sensor_fusion,
      const array<vector<double>, 5>& map_waypoints)
{

  // check current lane and init if needed
  if(currentLane == 0) {

    // calculate lane from d
    double d = localizationData.at(LOCAL_CAR_D);

    if(d <= -16.) currentLane = -4;
    else if(d <= -8.) currentLane = -3;
    else if(d <= -4.) currentLane = -2;
    else if(d <= -0.) currentLane = -1;
    else if(d >= 16.) currentLane = 4;
    else if(d >= 8.) currentLane = 3;
    else if(d >= 4.) currentLane = 2;
    else if(d >= 0.) currentLane = 1;
  }

  // for now, generate single trajectory
  // variable for returning the next (x, y) values
  array<vector<double>, 2> next_vals;

  switch(currentType) {
    case GEN_JMT:
      next_vals = std::move(trjGenerator.generateJMT());
      break;

    case GEN_SPLINE:
      next_vals = std::move(trjGenerator.generateSpline());
      break;

    case GEN_SIMPLE:
    default:
      next_vals = std::move(trjGenerator.generateSimple(referenceVelocity,
        currentLane, localizationData, map_waypoints));
      break;
  }

  return next_vals;
}

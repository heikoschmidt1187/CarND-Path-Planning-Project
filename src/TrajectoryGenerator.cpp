#include "TrajectoryGenerator.h"
#include "PathPlanner.h"

array<vector<double>, 2> TrajectoryGenerator::generateJMT()
{
  return array<vector<double>, 2>();
}

array<vector<double>, 2> TrajectoryGenerator::generateSpline()
{
  return array<vector<double>, 2>();
}

array<vector<double>, 2> TrajectoryGenerator::generateSimple(int lane,
  double refVel,
  const array<double, 6>& localizationData,
  const array<vector<double>, 5>& map_waypoints)
{
  // array to hold next values
  array<vector<double>, 2> next_vals;

  // simple trajectory just moves forward with 50 steps fixed for testing
  double increment_per_step = ((refVel * 1.6) / 3.6) * 0.02;

  for(int i = 0; i < 50; i++) {
    double next_s = localizationData.at(PathPlanner::LocalizationIndex::LOCAL_CAR_S) + (i + 1) * increment_per_step;
    double next_d = (2 + (lane - 1) * 4);

    vector<double> xy = getXY(next_s, next_d, map_waypoints.at(PathPlanner::MapWaypointsIndex::MWP_S), map_waypoints.at(PathPlanner::MapWaypointsIndex::MWP_X),
      map_waypoints.at(PathPlanner::MapWaypointsIndex::MWP_Y));

    next_vals.at(0).push_back(xy.at(0));
    next_vals.at(1).push_back(xy.at(1));
  }

  return next_vals;
}

#include <utility>
#include <iostream>

#include "PathPlanner.h"

array<vector<double>, 2> PathPlanner::plan(const SignalState& state, const Road& road)
{

  // get the current lane
  currentLane = Helpers::get_lane(road, state.egoState.d);

  predictor.predict(state, road);

  // for now, generate single trajectory
  // variable for returning the next (x, y) values
  return std::move(trjGenerator.generateSpline(currentLane, road.lane_speed_limit.at(currentLane), state));
}

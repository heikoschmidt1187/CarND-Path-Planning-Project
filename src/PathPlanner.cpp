#include <utility>
#include <iostream>

#include "PathPlanner.h"

array<vector<double>, 2> PathPlanner::plan(const SignalState& state, const Road& road)
{
  int target_lane;
  double target_velocity;

  predictor.predict(state, road);
  behavior.calc(state, road, target_lane, target_velocity);

  // for now, generate single trajectory
  // variable for returning the next (x, y) values
  return std::move(trjGenerator.generateSpline(target_lane, target_velocity, state));
}

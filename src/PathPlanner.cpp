#include <utility>
#include <iostream>

#include "PathPlanner.h"

array<vector<double>, 2> PathPlanner::plan(const SignalState& state)
{

  // check current lane and init if needed
  if(currentLane == 0) {

    // calculate lane from d
    double d = state.localizationData.at(SignalState::LOCAL_CAR_D);

    if(d <= -16.) currentLane = -4;
    else if(d <= -8.) currentLane = -3;
    else if(d <= -4.) currentLane = -2;
    else if(d <= -0.) currentLane = -1;
    else if(d >= 16.) currentLane = 3;
    else if(d >= 8.) currentLane = 2;
    else if(d >= 4.) currentLane = 1;
    else if(d >= 0.) currentLane = 0;

    targetLane = currentLane;
  }

  // for now, generate single trajectory
  // variable for returning the next (x, y) values
  array<vector<double>, 2> next_vals;

  switch(currentType) {
    case GEN_JMT:
      next_vals = std::move(trjGenerator.generateJMT(targetLane, referenceVelocity, state));
      break;

    case GEN_SPLINE:
      next_vals = std::move(trjGenerator.generateSpline(targetLane, referenceVelocity, state));
      break;

    case GEN_SIMPLE:
    default:
      next_vals = std::move(trjGenerator.generateSimple(targetLane, referenceVelocity, state));
      break;
  }

  return next_vals;
}

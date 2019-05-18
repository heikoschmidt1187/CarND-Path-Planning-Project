#include "TrajectoryGenerator.h"

#include <iostream>


array<vector<double>, 2> TrajectoryGenerator::generateJMT(const int& lane, const double& ref_velocity, const SignalState& state)
{
  return array<vector<double>, 2>();
}

array<vector<double>, 2> TrajectoryGenerator::generateSpline(const int& lane, const double& ref_velocity, const SignalState& state)
{
  return array<vector<double>, 2>();
}

array<vector<double>, 2> TrajectoryGenerator::generateSimple(const int& lane, const double& ref_velocity, const SignalState& state)
{
  // array to hold next values
  array<vector<double>, 2> next_vals;

  // simple trajectory just moves forward with 50 steps fixed for testing
  double increment_per_step = ((ref_velocity * 1.6) / 3.6) * 0.02;

  // we only want 50 points, so keep track of the last points
  for(int i = 0; i < 50; i++) {
    double next_s = state.localizationData.at(SignalState::LOCAL_CAR_S) + (i + 1) * increment_per_step;
    double next_d = (2 + lane * 4);

    vector<double> xy = Helpers::getXY(next_s, next_d, state.map_waypoints.at(SignalState::MWP_S), state.map_waypoints.at(SignalState::MWP_X),
      state.map_waypoints.at(SignalState::MWP_Y));

    next_vals.at(0).push_back(xy.at(0));
    next_vals.at(1).push_back(xy.at(1));
  }

  return next_vals;
}

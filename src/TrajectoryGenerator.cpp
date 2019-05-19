#include "TrajectoryGenerator.h"

#include <iostream>

void TrajectoryGenerator::updateCurrentBaseVelocity(double ref_velocity)
{
  // avoid too much change in jerk, so change the base velocity slowly (10m/s^2 acc/dev, 10m/s^3 jerk)
  // TODO: calculate

  /*
  if(currentBaseVelocity < ref_velocity) {
    currentBaseVelocity += 0.1;
  } else {
    currentBaseVelocity -= 0.1;
  }
  */

  currentBaseVelocity = ref_velocity;
}

array<vector<double>, 2> TrajectoryGenerator::generateJMT(const int& lane, const double& ref_velocity, const SignalState& state)
{
  // set the velocity for this calculation cycle
  // TODO: take care when generating multiple trajectories in one cycle! - maybe better in PathPlanner.plan() ?
  updateCurrentBaseVelocity(ref_velocity);

  return array<vector<double>, 2>();
}

array<vector<double>, 2> TrajectoryGenerator::generateSpline(const int& lane, const double& ref_velocity, const SignalState& state)
{
  // set the velocity for this calculation cycle
  // TODO: take care when generating multiple trajectories in one cycle! - maybe better in PathPlanner.plan() ?
  updateCurrentBaseVelocity(ref_velocity);

  // create list of widely spread points to create spline from
  vector<double> ptsx, ptsy;

  // reference state
  double ref_x = state.egoState.x;
  double ref_y = state.egoState.y;
  double ref_yaw = state.egoState.yaw;

  // previous path size
  size_t prev_size = state.prev_path_x.size();

  // if previous path is nearly empty, use the current car posisiton as reference
  if(prev_size < 2) {
    // calculate a pseudo-previous car point according to the current position and yaw angle
    double prev_car_x = ref_x - cos(ref_yaw);
    double prev_car_y = ref_y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ref_y);

  } else {
    // we have at least two points in the last path, so take them to calculate the tangent for the next points
    ref_x = state.prev_path_x.at(prev_size - 1);
    ref_y = state.prev_path_y.at(prev_size - 1);

    double ref_x_prev = state.prev_path_x.at(prev_size - 2);
    double ref_y_prev = state.prev_path_y.at(prev_size - 2);

    // change the yaw angle to the tangent of the last two path points to get a smooth path transition
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // calculate three sparse wide range waypoints (30m, 60m, 90m) in Frenet as base for the spline calculation
  vector<double> next_wp0 = Helpers::getXY(state.egoState.s + 30, (2 + 4 * lane),
    state.map_waypoints.at(SignalState::MWP_S), state.map_waypoints.at(SignalState::MWP_X), state.map_waypoints.at(SignalState::MWP_Y));

  vector<double> next_wp1 = Helpers::getXY(state.egoState.s + 60, (2 + 4 * lane),
    state.map_waypoints.at(SignalState::MWP_S), state.map_waypoints.at(SignalState::MWP_X), state.map_waypoints.at(SignalState::MWP_Y));

  vector<double> next_wp2 = Helpers::getXY(state.egoState.s + 90, (2 + 4 * lane),
    state.map_waypoints.at(SignalState::MWP_S), state.map_waypoints.at(SignalState::MWP_X), state.map_waypoints.at(SignalState::MWP_Y));

    ptsx.push_back(next_wp0.at(0));
    ptsx.push_back(next_wp1.at(0));
    ptsx.push_back(next_wp2.at(0));

    ptsy.push_back(next_wp0.at(1));
    ptsy.push_back(next_wp1.at(1));
    ptsy.push_back(next_wp2.at(1));

    // shift the reference data to local car coordinates for easier calculation (position at origin, angle 0)
    for(size_t i = 0; i < ptsx.size(); ++i) {

      double shift_x = ptsx.at(i) - ref_x;
      double shift_y = ptsy.at(i) - ref_y;

      ptsx.at(i) = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
      ptsy.at(i) = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // create a spline
    tk::spline s;

    // set the spline points
    s.set_points(ptsx, ptsy);

    // array to hold next values
    array<vector<double>, 2> next_vals;

    // push back the previous path points first
    for(size_t i = 0; i < prev_size; ++i) {
      next_vals.at(0).push_back(state.prev_path_x.at(i));
      next_vals.at(1).push_back(state.prev_path_y.at(i));
    }

    // break up path to travel in desired reference velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

    double x_add_on = 0;

    // fill up the rest of the path up to 50 points in total
    for(size_t i = 0; i <= (LOOKAHEAD_POINTS - prev_size); ++i) {

      // calculate needed segments (ref_velocity in mph)
      double N = (target_dist / (.02 * currentBaseVelocity));
      double x_point = x_add_on + (target_x) / N;
      double y_point = s(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      // rotate back to normal, map coordinates
      x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
      y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

      x_point += ref_x;
      y_point += ref_y;

      next_vals.at(0).push_back(x_point);
      next_vals.at(1).push_back(y_point);
    }

    return next_vals;
}

array<vector<double>, 2> TrajectoryGenerator::generateSimple(const int& lane, const double& ref_velocity, const SignalState& state)
{
  // set the velocity for this calculation cycle
  // TODO: take care when generating multiple trajectories in one cycle! - maybe better in PathPlanner.plan() ?
  updateCurrentBaseVelocity(ref_velocity);

  // array to hold next values
  array<vector<double>, 2> next_vals;

  // simple trajectory just moves forward with 50 steps fixed for testing
  double increment_per_step = ((currentBaseVelocity * 1.6) / 3.6) * 0.02;

  // we only want 50 points, so keep track of the last points
  for(int i = 0; i < LOOKAHEAD_POINTS; i++) {
    double next_s = state.egoState.s + (i + 1) * increment_per_step;
    double next_d = (2 + lane * 4);

    vector<double> xy = Helpers::getXY(next_s, next_d, state.map_waypoints.at(SignalState::MWP_S), state.map_waypoints.at(SignalState::MWP_X),
      state.map_waypoints.at(SignalState::MWP_Y));

    next_vals.at(0).push_back(xy.at(0));
    next_vals.at(1).push_back(xy.at(1));
  }

  return next_vals;
}

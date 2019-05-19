#include "BehaviorPlanner.h"

void BehaviorPlanner::calc(const SignalState& state, const Road& road, int& target_lane, double& target_velocity)
{
  std::cout << "***" << std::endl;

  // for now as first simple approach: check if car is within specific range in front
  // and adapt speed
  double distance_front_car = 999.;
  double front_car_speed = 999.;
  double closest_car_id = 999.;

  // own lane
  int current_lane = Helpers::get_lane(road, state.egoState.d);
  target_lane = current_lane;

  // set reference lane speed
  target_velocity = road.lane_speed_limit.at(current_lane);

  // project own s coordinate into the future of the existing path
  double car_pred_s = (state.prev_path_x.size() == 0) ? state.egoState.s : state.end_path_Frenet.at(SignalState::END_PATH_S);

  // loop over detected cars
  for(const auto& c : state.sensor_fusion) {
    // check if the car is in our lane by comparing d and if it's in front of us
    if((c.at(6) > 0) && (c.at(5) > state.egoState.s) && (current_lane == Helpers::get_lane(road, c.at(6)))) {

      // calculate the car's speed for further calculations on constant velocity model
      double c_speed = sqrt(pow(c.at(3), 2) + pow(c.at(4), 2));

      // predict the car's future position at the current path end
      double c_s = c.at(5) + static_cast<double>(state.prev_path_x.size()) * 0.02 * c_speed;

      // check the predicted distance in the future and remenber closest car
      double pred_distance = c_s - car_pred_s;

      if(pred_distance < distance_front_car) {
        distance_front_car = pred_distance;
        front_car_speed = c_speed;
        closest_car_id = c.at(0);
      }
    }
  }

  // if the closest car is below a specific threshold in the future, slow down or speed up
  if(distance_front_car < 30) {

    std::cout << "Closest car id " << closest_car_id
              << " will be at dist=" << distance_front_car
              << " at speed " << front_car_speed
              << " in seconds=" << static_cast<double>(state.prev_path_x.size()) * 0.02
              << std::endl;

    // slow down current velocity
    target_velocity = std::max(1., state.egoState.speed - 1);
  } else {
    target_velocity = std::min(road.lane_speed_limit.at(current_lane), state.egoState.speed + 1);
  }

  std::cout << "Target velocity: " << target_velocity << std::endl;
}

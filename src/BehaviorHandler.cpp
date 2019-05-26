#include <iostream>

#include "BehaviorHandler.h"
#include "Parameter.h"

BehaviorHandler::BehaviorHandler()
  : current_fsm_state(s_KL)
  , prev_lane(-1)
  , lane_change_cycles(0)
{
  // -1 are cars with no valid lane detected
  for(int i = -1; i < Parameter::k_lane_count; ++i)
    lanes.insert(std::make_pair(i, Lane()));
}

BehaviorTarget BehaviorHandler::plan(const Car::State& s_state, const Car::State& d_state, const std::vector<Car>& other_cars)
{
  // map other cars to lanes for further processing
  updateCarMap(s_state.position, other_cars);

  // for debug reasons, plot values of each lane
  /*
  std::cout << "**** Lanes ****" << std::endl;
  for(int i = -1; i < Parameter::k_lane_count; ++i) {
    std::cout << "Lane " << i << ": " << lanes[i].vehicles.size() << std::endl;
    std::cout << "Closest Front: " << lanes[i].closest_front_id <<
      " Distance Front: " << lanes[i].distance_front << std::endl;
    std::cout << "Closest Rear: " << lanes[i].closest_rear_id <<
      " Distance Rear: " << lanes[i].distance_rear << std::endl;
    std::cout << "Lane Speed: " << lanes[i].speed << std::endl << std::endl;
  }
  */

  // run the state machine
  BehaviorTarget target;

  switch(current_fsm_state) {
    case s_KL:
      target = keepLane(s_state, d_state, other_cars);
      break;

    case s_LCL:
      target = laneChangeLeft(s_state, d_state, other_cars);
      break;

    case s_LCR:
      target = laneChangeRight(s_state, d_state, other_cars);
      break;
  }

  // remember lane for next cycle in case we want a lane change
  prev_lane = Car::calcLane(d_state.position);

  return target;
}

BehaviorTarget BehaviorHandler::keepLane(const Car::State& s_state, const Car::State& d_state, const std::vector<Car>& other_cars)
{
  std::cout << "STATE: KEEP LANE" << std::endl;

  BehaviorTarget target;
  target.need_fast_reaction = false;

  // for now: keep in lane
  double target_speed = s_state.velocity;
  int predicted_lane = 1; //Car::calcLane(d_state.position);
  int target_lane = predicted_lane;

  // if no car is in my lane or if it's far away, stay in lane
  if(   (lanes[predicted_lane].vehicles.empty() == true)
    ||  (lanes[predicted_lane].distance_front >
      (3 * Parameter::k_prediction_time * s_state.velocity))) {

    std::cout << "no car in front" << std::endl;

    // increase speed
    target_speed = static_cast<double>(Parameter::k_speed_limit - Parameter::k_speed_buffer);

  } else {

    // there's a car in front of us that may force us to speed up faster or even slow
    // down from possible speed limit --> check if there's a faster lane
    // for now: just compare, TODO: cost function!
    if(predicted_lane == 0) {
      // left-most lane, only change to next right is possible
      if(lanes[predicted_lane + 1].speed > lanes[predicted_lane].speed) {
        current_fsm_state = s_LCR;
      }
    } else if(predicted_lane == (Parameter::k_lane_count - 1)) {
      // right-most lane, only change to next left is possible
      if(lanes[predicted_lane - 1].speed > lanes[predicted_lane].speed) {
        current_fsm_state = s_LCL;
      }
    } else {
      // check for faster lane neighbor lane
      int lane_index = predicted_lane;

      if(lanes[lane_index + 1].speed > lanes[lane_index].speed) {
        lane_index += 1;
      } else {
        lane_index -= 1;
      }

      // compare targeting lane
      if(lanes[lane_index].speed > lanes[predicted_lane].speed) {
        current_fsm_state = (lane_index < predicted_lane) ? s_LCL : s_LCR;
      }
    }

    std::cout << "State after evaluation: " << current_fsm_state << std::endl;

    // TODO: refactor - there must be a better implementation!
    // depending on if we are already in the fastest lane, we still may need to
    // adjust are speed to preceting traffic
    if(current_fsm_state == s_KL) {
      if(lanes[predicted_lane].distance_front > (Parameter::k_prediction_time * s_state.velocity)) {

        std::cout << "car in front - safe distance" << std::endl;

        // increase speed
        target_speed = static_cast<double>(Parameter::k_speed_limit - Parameter::k_speed_buffer);

      } else {

        target.need_fast_reaction = true;

        std::cout << "car in front - slow down " << lanes[target_lane].closest_front_speed << std::endl;

        // decrease speed
        target_speed = lanes[target_lane].closest_front_speed - 1;
      }
    } else {
      lane_change_cycles = 0;
      target_lane += (current_fsm_state == s_LCL) ? -1 : 1;
    }
  }

  // TEST always stay in state
  current_fsm_state = s_KL;

  target.speed = target_speed;
  target.lane = target_lane;

  return target;
}

BehaviorTarget BehaviorHandler::laneChangeLeft(const Car::State& s_state, const Car::State& d_state, const std::vector<Car>& other_cars)
{
  std::cout << "STATE: LANE CHANGE LEFT" << std::endl;

  int predicted_lane = Car::calcLane(d_state.position);
  int target_lane = predicted_lane;
  double target_speed = s_state.velocity;

  lane_change_cycles++;

  // check if lane chane is finished
  if(predicted_lane != prev_lane) {
    std::cout << "car is in new lane" << std::endl;
    target_lane = predicted_lane;
  } else {
    target_lane = predicted_lane - 1;

    // if car is in the same lane and the cycles are over, switch to KL
    if(lane_change_cycles >= Parameter::k_min_lane_change_cycles) {
      std::cout << "Switch to KL" << std::endl;
      lane_change_cycles = 0;
      current_fsm_state = s_KL;
    }
  }

  // for now: maintain the speed during lane change

  BehaviorTarget target;
  target.need_fast_reaction = false;
  target.speed = target_speed;
  target.lane = target_lane;

  return target;
}

BehaviorTarget BehaviorHandler::laneChangeRight(const Car::State& s_state, const Car::State& d_state, const std::vector<Car>& other_cars)
{
  std::cout << "STATE: LANE CHANGE RIGHT" << std::endl;

  int predicted_lane = Car::calcLane(d_state.position);
  int target_lane = predicted_lane;
  double target_speed = s_state.velocity;

  lane_change_cycles++;

  // check if lane chane is finished
  if(predicted_lane != prev_lane) {
    std::cout << "car is in new lane" << std::endl;
    target_lane = predicted_lane;
  } else {
    target_lane = predicted_lane + 1;

    // if car is in the same lane and the cycles are over, switch to KL
    if(lane_change_cycles >= Parameter::k_min_lane_change_cycles) {
      std::cout << "Switch to KL" << std::endl;
      lane_change_cycles = 0;
      current_fsm_state = s_KL;
    }
  }

  // for now: maintain the speed during lane change
  BehaviorTarget target;
  target.need_fast_reaction = false;
  target.speed = target_speed;
  target.lane = target_lane;

  return target;
}

void BehaviorHandler::updateCarMap(const double pred_s, const std::vector<Car>& other_cars)
{
  // clear the lanes
  for(int i = -1; i < Parameter::k_lane_count; ++i)
    lanes[i] = Lane();

  // sort the other cars
  for(const auto& o : other_cars)
    lanes[o.getLane()].vehicles.push_back(o);

  // update the lane data
  for(int i = -1; i < Parameter::k_lane_count; ++i)
    lanes[i].update(pred_s);
}

void BehaviorHandler::Lane::update(const double s)
{
  closest_front_id = -1;
  closest_rear_id = -1;
  distance_front = std::numeric_limits<double>::infinity();
  distance_rear = std::numeric_limits<double>::infinity();
  speed = Parameter::k_speed_limit;

  for(const auto& v : vehicles) {

    // check if car is in front of behind
    if(v.getS() >= s) {

      // check if car is closer
      if((v.getS() - s) < distance_front) {
        distance_front = v.getS() - s;
        closest_front_id = v.getId();
        closest_front_speed = v.getSpeed();

        // lane speed defined by slowest car in front
        if(v.getSpeed() < speed) {
          speed = v.getSpeed();
        }
      }

    } else {

      // check if car is closer
      if((s - v.getS()) < distance_rear) {
        distance_rear = s - v.getS();
        closest_rear_id = v.getId();
        closest_rear_speed = v.getSpeed();
      }
    }
  }
}

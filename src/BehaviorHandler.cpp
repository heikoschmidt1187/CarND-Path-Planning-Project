#include <iostream>

#include "BehaviorHandler.h"
#include "Parameter.h"

BehaviorHandler::BehaviorHandler()
  : current_fsm_state(s_KL)
  , prev_lane(-1)
  , target_lane(-1)
  , lane_change_cycles(0)
{
  // -1 are cars with no valid lane detected
  for(int i = -1; i < Parameter::k_lane_count; ++i)
    lanes.insert(std::make_pair(i, Lane()));
}

BehaviorTarget BehaviorHandler::plan(const Car& ego, const Car::State& s_state, const double future_time, const Car::State& d_state, const std::vector<Car>& other_cars)
{
  // map other cars to lanes for further processing
  //updateCarMap(s_state.position, other_cars);
  updateCarMap(ego.getS(), other_cars);

  // on first call, init target and prev_lane
  if(target_lane < 0) {
    target_lane = Car::calcLane(d_state.position);
    prev_lane = target_lane;
  }

  // for debug reasons, plot values of each lane
  std::cout << "**** Lanes ****" << std::endl;
  for(int i = -1; i < Parameter::k_lane_count; ++i) {
    std::cout << "Lane " << i << ": " << lanes[i].vehicles.size() << std::endl;
    std::cout << "Closest Front: " << lanes[i].closest_front_id <<
      " Distance Front: " << lanes[i].distance_front <<
      " Front Speed: " << lanes[i].closest_front_speed << std::endl;
    std::cout << "Closest Rear: " << lanes[i].closest_rear_id <<
      " Distance Rear: " << lanes[i].distance_rear <<
      " Rear Speed: " << lanes[i].closest_rear_speed << std::endl;
    std::cout << "Lane Speed: " << lanes[i].speed << std::endl << std::endl;
  }

  // run the state machine
  BehaviorTarget target;

  switch(current_fsm_state) {
    case s_KL:
      target = keepLane(s_state, future_time, d_state, other_cars);
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

std::vector<double> BehaviorHandler::getLaneCost(const int lane, const double velocity, const double future_time)
{
  std::vector<double> costs(Parameter::k_lane_count);

  // lane cost is calculated depending on the lane (prefer middle lane if possible) and the lane's speed
  // where fater lanes are better

  int mid_lane = Parameter::k_lane_count / 2;

  std::cout << "Lane costs: " << std::endl;

  for(size_t i = 0; i < costs.size(); ++i) {

    ////// lane number //////
    costs.at(i) = Parameter::k_lane_number_weight * std::fabs(static_cast<double>(i) - static_cast<double>(mid_lane)) / Parameter::k_lane_count;

    ////// lane speed //////
    // if nothing in front, the speed cost is 0 and therefore optimal
    // HINT: predict future position of other car due to constant velocity for now
    double future_dist = (lanes[i].distance_front + lanes[i].closest_front_speed * future_time) - (velocity * future_time);

    if(   (lanes[i].vehicles.empty() == false)
      &&  (future_dist <= (Parameter::k_gap_buffer_time * Parameter::k_prediction_time * velocity))
      &&  (lanes[i].speed < velocity)) {

      double t_speed = Parameter::k_speed_limit - Parameter::k_speed_buffer;
      costs.at(i) += Parameter::k_lane_speed_weight * ((t_speed - lanes[i].speed) / t_speed);
    }

    std::cout << "Lane " << i << ": " << costs.at(i) << std::endl;
  }

  return costs;
}

BehaviorTarget BehaviorHandler::keepLane(const Car::State& s_state, const double future_time, const Car::State& d_state, const std::vector<Car>& other_cars)
{
  std::cout << "STATE: KEEP LANE" << std::endl;
  current_fsm_state = s_KL;

  BehaviorTarget target;
  target.need_fast_reaction = false;

  // for now: keep in lane
  int predicted_lane = Car::calcLane(d_state.position);
  target_lane = predicted_lane;

  // check for lane changes acording to lane costs
  // get the cost for each lane
  auto lane_cost = getLaneCost(predicted_lane, s_state.velocity, future_time);

  // check for lane change safety
  auto lcl_safe = isLaneChangeSafe(s_LCL, s_state.velocity, predicted_lane);
  auto lcr_safe = isLaneChangeSafe(s_LCR, s_state.velocity, predicted_lane);

  std::cout << "Lane change safe - left: " << lcl_safe << ", right: " << lcr_safe << std::endl;

  if(predicted_lane == 0) {
    // left-most lane, only compare right lane to the current
    if(   (lane_cost.at(predicted_lane) > lane_cost.at(predicted_lane + 1))
      &&  (lcr_safe == true)) {
      current_fsm_state = s_LCR;
    }
  } else if(predicted_lane == (Parameter::k_lane_count - 1)) {
    // right-most lane, only compare left lane to the current
    if(   (lane_cost.at(predicted_lane) > lane_cost.at(predicted_lane - 1)
      &&  (lcl_safe == true))) {
      current_fsm_state = s_LCL;
    }
  } else {
    // check both neighbor lanes - try to change to better lane first
    if(lane_cost.at(predicted_lane - 1) <= lane_cost.at(predicted_lane + 1)) {

      if(   (lane_cost.at(predicted_lane - 1) < lane_cost.at(predicted_lane))
        &&  (lcl_safe == true)) {
        current_fsm_state = s_LCL;
      } else if(  (lane_cost.at(predicted_lane + 1) < lane_cost.at(predicted_lane))
              &&  (lcr_safe == true)) {
        current_fsm_state = s_LCR;
      }

    } else {

      if(   (lane_cost.at(predicted_lane + 1) < lane_cost.at(predicted_lane))
        &&  (lcr_safe == true)) {
        current_fsm_state = s_LCR;
      } else if(   (lane_cost.at(predicted_lane - 1) < lane_cost.at(predicted_lane))
              &&   (lcl_safe == true)) {
        current_fsm_state = s_LCL;
      }
    }
  }

  std::cout << "State after evaluation: " << current_fsm_state << std::endl;

  if(current_fsm_state == s_KL) {
   std::cout << "Keeping lane ";

  // speed depends on if there's a car in front
  if(   (lanes[target_lane].vehicles.empty() == true)
    ||  (lanes[target_lane].distance_front > (Parameter::k_gap_buffer_time * Parameter::k_prediction_time * s_state.velocity))
    ||  (lanes[target_lane].distance_rear > (Parameter::k_gap_buffer_time * Parameter::k_prediction_time * s_state.velocity))) {

    target.speed = Parameter::k_speed_limit - Parameter::k_speed_buffer;
  } else {
    target.speed = lanes[target_lane].speed - 1;

    // if there's a car directly in front if us (directly behind in predicted future) we need a fast reaction
    // and slow down faster
    if(lanes[target_lane].distance_rear <= (Parameter::k_prediction_time * s_state.velocity)) {
      target.need_fast_reaction = true;
      target.speed -= 1;
    }
  }

  } else if(current_fsm_state == s_LCL) {
    std::cout << "Changing to left lane ";
    target_lane = predicted_lane - 1;
    target.speed = lanes[target_lane].speed;
  } else if(current_fsm_state == s_LCR) {
    std::cout << "Changing to right lane ";
    target_lane = predicted_lane + 1;
    target.speed = lanes[target_lane].speed;
  } else {
    std::cout << "no state..." << std::endl;
    target.speed = 0.;
  }


  // TODO: lane change left safe check
  // TODO: avoid useless changes
  target.speed = std::min(target.speed, static_cast<double>(Parameter::k_speed_limit - Parameter::k_speed_buffer));
  target.lane = target_lane;

  std::cout << target.lane << " at speed " << target.speed << std::endl;

  return target;
}

BehaviorTarget BehaviorHandler::laneChangeLeft(const Car::State& s_state, const Car::State& d_state, const std::vector<Car>& other_cars)
{
  std::cout << "STATE: LANE CHANGE LEFT" << std::endl;

  int predicted_lane = Car::calcLane(d_state.position);
  double target_speed = s_state.velocity;

  lane_change_cycles++;

  // lange change is finished if target and current lane are equal and lateral speed is small
  if((predicted_lane == target_lane) && (std::fabs(d_state.velocity) < 0.3)) {
    std::cout << "Lane change left finished, back to KL" << std::endl;
    current_fsm_state = s_KL;
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
  double target_speed = s_state.velocity;

  lane_change_cycles++;

  // lange change is finished if target and current lane are equal and lateral speed is small
  if((predicted_lane == target_lane) && (std::fabs(d_state.velocity) < 0.3)) {
    std::cout << "Lane change right finished, back to KL" << std::endl;
    current_fsm_state = s_KL;
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
  std::cout << "Update car map " << pred_s << std::endl;
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

    // check if car is in front or behind
    if(v.getS() >= s) {

      // check if car is closer
      if(std::fabs(v.getS() - s) < distance_front) {
        distance_front = std::fabs(v.getS() - s);
        closest_front_id = v.getId();
        closest_front_speed = v.getSpeed();

        // lane speed defined by slowest car in front
        if(v.getSpeed() < speed) {
          speed = v.getSpeed();
        }
      }

    } else {

      // check if car is closer
      if(std::fabs(s - v.getS()) < distance_rear) {
        distance_rear = std::fabs(s - v.getS());
        closest_rear_id = v.getId();
        closest_rear_speed = v.getSpeed();
      }
    }
  }
}

bool BehaviorHandler::isLaneChangeSafe(const FsmState state, const double velocity, const int lane)
{
  // check wrong parameters
  if((lane < 0) || (lane >= Parameter::k_lane_count) || (state == s_KL)) {
    std::cout << "Wrong questioning parameter for lane change safety" << std::endl;
    return false;
  }

  // check lane borders
  if((state == s_LCL) && (lane == 0)) {
    // no left lane
    return false;

  } else if((state == s_LCR) && (lane == (Parameter::k_lane_count - 1))) {
    // no right lane
    return false;

  } else {
    // check for gap in target lane and speed of nearby cars
    int check_lane = (state == s_LCL) ? lane - 1 : lane + 1;

    std::cout << "LC safe flags: ";
    std::cout << (lanes[check_lane].distance_rear > Parameter::k_gap_buffer_time * lanes[check_lane].closest_rear_speed)<< " ";
    std::cout << (lanes[check_lane].closest_rear_speed <= 1.5 * velocity) << " ";
    std::cout << (lanes[check_lane].distance_front > Parameter::k_gap_buffer_time * velocity) << std::endl;

    return (  (lanes[check_lane].distance_rear > Parameter::k_gap_buffer_time * lanes[check_lane].closest_rear_speed)
          &&  (lanes[check_lane].closest_rear_speed <= 1.5 * velocity)
          &&  (lanes[check_lane].distance_front > Parameter::k_gap_buffer_time * velocity));
  }
}

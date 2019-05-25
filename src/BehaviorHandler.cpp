#include <iostream>

#include "BehaviorHandler.h"
#include "Parameter.h"

BehaviorHandler::BehaviorHandler()
  : current_fsm_state(s_KL)
{
  // -1 are cars with no valid lane detected
  for(int i = -1; i < Parameter::k_lane_count; ++i)
    lanes.insert(std::make_pair(i, Lane()));
}

Car BehaviorHandler::plan(const Car& prediction, const std::vector<Car>& other_cars)
{
  // map other cars to lanes for further processing
  updateCarMap(prediction.getS(), other_cars);

  // for debug reasons, plot values of each lane
  std::cout << "**** Lanes ****" << std::endl;
  for(int i = -1; i < Parameter::k_lane_count; ++i) {
    std::cout << "Lane " << i << ": " << lanes[i].vehicles.size() << std::endl;
    std::cout << "Closest Front: " << lanes[i].closest_front_id <<
      " Distance Front: " << lanes[i].distance_front << std::endl;
    std::cout << "Closest Rear: " << lanes[i].closest_rear_id <<
      " Distance Rear: " << lanes[i].distance_rear << std::endl;
    std::cout << "Lane Speed: " << lanes[i].speed << std::endl << std::endl;
  }

  // run the state machine
  Car target;

  switch(current_fsm_state) {
    case s_KL:
      target = keepLane(prediction, other_cars);
      break;

    case s_LCL:
      target = laneChangeLeft(prediction, other_cars);
      break;

    case s_LCR:
      target = laneChangeRight(prediction, other_cars);
      break;
  }

  return target;
}

Car BehaviorHandler::keepLane(const Car& prediction, const std::vector<Car>& other_cars)
{
  std::cout << "STATE: KEEP LANE" << std::endl;

  // for now: keep in lane
  Car target(prediction.getId());

  // get the closest other car in front of out prediction
  double distance = std::numeric_limits<double>::infinity();
  double front_speed = 0.;

  for(const auto& o : other_cars) {
    /*
    std::cout << "*** Object" << o.getId() << std::endl;
    std::cout << o.getLane() << ", " << o.getS() << ", " << o.getD() << ", " << o.getSpeed() << std::endl;
    std::cout << prediction.getLane() << ", " << prediction.getS() << ", " << prediction.getD() << ", " << prediction.getSpeed() << std::endl;
    */

    if((o.getLane() == prediction.getLane()) && (o.getS() > prediction.getS())) {
      if(distance > (o.getS() - prediction.getS())) {
        distance = o.getS() - prediction.getS();
        front_speed = o.getSpeed();
      }
    }
  }

  std::cout << "--DISTANCE " << distance << std::endl;

  double target_speed = prediction.getSpeed();

  // we predict 2s into the future, so the critical distance to react is velocity * time
  if(distance > 3 * Parameter::k_prediction_time * target_speed) {
    std::cout << "no car in front" << std::endl;

    // increase speed
    target_speed = std::min(static_cast<double>(Parameter::k_speed_limit - Parameter::k_speed_buffer),
      prediction.getSpeed() + 5);

  } else if(distance > (target_speed * Parameter::k_prediction_time)) {

    std::cout << "car in front - safe distance" << std::endl;

    // increase speed
    target_speed = std::min(static_cast<double>(Parameter::k_speed_limit - Parameter::k_speed_buffer),
      prediction.getSpeed() + 1);

  } else {
    std::cout << "car in front - slow down " << front_speed << std::endl;

    // decrease speed
    target_speed = std::max(front_speed - 1, target_speed - 1);
  }

  target.update(prediction.getS() + target_speed * Parameter::k_prediction_time,
    (2 + prediction.getLane() * 4), target_speed);

  return target;
}

Car BehaviorHandler::laneChangeLeft(const Car& prediction, const std::vector<Car>& other_cars)
{
  return Car();
}

Car BehaviorHandler::laneChangeRight(const Car& prediction, const std::vector<Car>& other_cars)
{
  return Car();
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
      }
    }
  }
}

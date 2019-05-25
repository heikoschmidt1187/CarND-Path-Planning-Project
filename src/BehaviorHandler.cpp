#include <iostream>
#include <limits>

#include "BehaviorHandler.h"
#include "Parameter.h"

BehaviorHandler::BehaviorHandler()
  : currentState(s_KL)
{}

Car BehaviorHandler::plan(const Car& prediction, const std::vector<Car>& other_cars)
{
  // for now, just check for traffic and stay in lane
  Car target(prediction.getId());

  // get current lane
  int lane = prediction.getLane();

  // get the closest other car in front of out prediction
  double distance = std::numeric_limits<double>::infinity();
  double front_speed = 0.;

  std::cout << "Other cars: " << other_cars.size() << std::endl;

  for(const auto& o : other_cars) {
    std::cout << "*** Object" << o.getId() << std::endl;
    std::cout << o.getLane() << ", " << o.getS() << ", " << o.getD() << ", " << o.getSpeed() << std::endl;
    std::cout << lane << ", " << prediction.getS() << ", " << prediction.getD() << ", " << prediction.getSpeed() << std::endl;

    if((o.getLane() == lane) && (o.getS() > prediction.getS())) {
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
    (2 + lane * 4), target_speed);

  return target;
}

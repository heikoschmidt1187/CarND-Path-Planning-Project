#ifndef BEHAVIOR_HANDLER_H_
#define BEHAVIOR_HANDLER_H_

#include <limits>
#include <vector>
#include <map>

#include "Parameter.h"
#include "Car.h"

class BehaviorHandler {

private:
  // behavior Fsm states
  enum FsmState {
    s_KL,       // keep lane
    s_LCL,      // lane change left
    s_LCR       // lane change right
  };

  struct Lane {
    Lane()
      : closest_front_id(-1), distance_front(std::numeric_limits<double>::infinity())
      , closest_rear_id(-1), distance_rear(std::numeric_limits<double>::infinity())
      , speed(Parameter::k_speed_limit)
    {}

    void update(const double s);

    std::vector<Car> vehicles;    // list of vehicles in lane

    int closest_front_id;         // id of closest car in front
    double distance_front;        // distance to closest car in front

    int closest_rear_id;          // id of closest car in rear
    double distance_rear;         // distance to closest rear

    double speed;            // lane speed
  };

public:
  // C'tors
  BehaviorHandler();

  // plan and return target car prediction
  Car plan(const Car& prediction, const std::vector<Car>& other_cars);

private:
  Car keepLane(const Car& prediction, const std::vector<Car>& other_cars);
  Car laneChangeLeft(const Car& prediction, const std::vector<Car>& other_cars);
  Car laneChangeRight(const Car& prediction, const std::vector<Car>& other_cars);

  void updateCarMap(const double pred_s, const std::vector<Car>& other_cars);

private:
  FsmState current_fsm_state;

  std::map<int, Lane> lanes;
};

#endif /* BEHAVIOR_HANDLER_H_ */

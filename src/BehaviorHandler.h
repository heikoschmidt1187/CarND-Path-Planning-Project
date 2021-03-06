#ifndef BEHAVIOR_HANDLER_H_
#define BEHAVIOR_HANDLER_H_

#include <limits>
#include <vector>
#include <map>

#include "Parameter.h"
#include "Car.h"

struct BehaviorTarget {
  bool need_fast_reaction;
  double speed;
  int lane;
};

class BehaviorHandler {

private:
  // behavior Fsm states
  enum FsmState {
    s_KL = 0,   // keep lane
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
    double closest_front_speed;   // speed of closest car in front

    int closest_rear_id;          // id of closest car in rear
    double distance_rear;         // distance to closest rear
    double closest_rear_speed;   // speed of closest car rear

    double speed;            // lane speed
  };

public:
  // C'tors
  BehaviorHandler();

  // plan and return target car prediction
  BehaviorTarget plan(const Car& ego, const Car::State& s_state, const double future_time, const Car::State& d_state, const std::vector<Car>& other_cars);

private:
  BehaviorTarget keepLane(const Car::State& s_state, const double future_time, const Car::State& d_state, const std::vector<Car>& other_cars);
  BehaviorTarget laneChangeLeft(const Car::State& s_state, const Car::State& d_state, const std::vector<Car>& other_cars);
  BehaviorTarget laneChangeRight(const Car::State& s_state, const Car::State& d_state, const std::vector<Car>& other_cars);

  void updateCarMap(const double pred_s, const std::vector<Car>& other_cars);
  std::vector<double> getLaneCost(const int lane, const double velocity, const double future_time);
  bool isLaneChangeSafe(const FsmState state, const double velocity, const int lane);

private:
  FsmState current_fsm_state;

  std::map<int, Lane> lanes;

  int prev_lane;
  int target_lane;

  int lane_change_cycles;
};

#endif /* BEHAVIOR_HANDLER_H_ */

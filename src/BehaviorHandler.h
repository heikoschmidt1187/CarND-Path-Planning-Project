#ifndef BEHAVIOR_HANDLER_H_
#define BEHAVIOR_HANDLER_H_

#include <vector>

#include "Car.h"

class BehaviorHandler {

private:
  // behavior Fsm states
  enum FsmState {
    s_KL,       // keep lane
    s_PLCL,     // prepare lane change left
    s_PLCR,     // prepare lane change right
    s_LCL,      // lane change left
    s_LCR       // lane change right
  };

public:
  // C'tors
  BehaviorHandler();

  // plan and return target car prediction
  Car plan(const Car& prediction, const std::vector<Car>& other_cars);

private:
  FsmState currentState;
};

#endif /* BEHAVIOR_HANDLER_H_ */

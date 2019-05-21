#ifndef _BEHAVIORPLANNER_H_
#define _BEHAVIORPLANNER_H_

#include "helpers.h"

#include <vector>
#include <tuple>

class BehaviorPlanner {
public:
  BehaviorPlanner()
    : currentFsmState(FSM_KEEP_LANE)
    , speed_P_factor(0.5)
  {}

  void calc(const SignalState& state, const Road& road, int& target_lane, double& target_velocity);

private:
  enum FsmState {
    FSM_KEEP_LANE,
    FSM_PREP_CHANGE_LANE,
    FSM_CHANGE_LANE
  };

  FsmState currentFsmState;

  double speed_P_factor;

};

#endif /* end of include guard: _BEHAVIORPLANNER_H_ */

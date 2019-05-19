#ifndef _BEHAVIORPLANNER_H_
#define _BEHAVIORPLANNER_H_


class BehaviorPlanner {
public:
  BehaviorPlanner()
    : currentFsmState(FSM_KEEP_LANE)
  {}

  void plan();

private:
  enum FsmState {
    FSM_KEEP_LANE,
    FSM_PREP_CHANGE_LANE,
    FSM_CHANGE_LANE
  };

  FsmState currentFsmState;
};

#endif /* end of include guard: _BEHAVIORPLANNER_H_ */

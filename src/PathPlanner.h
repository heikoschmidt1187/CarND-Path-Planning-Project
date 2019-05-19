#ifndef _PATHPLANNER_H_
#define _PATHPLANNER_H_

#include "TrajectoryGenerator.h"
#include "Predictor.h"
#include "BehaviorPlanner.h"

#include <vector>
#include <array>

using namespace std;

class PathPlanner {
public:
  PathPlanner()
    : currentLane(0)
    , targetLane(0)
    , referenceVelocity(49.5)
  {}

  array<vector<double>, 2> plan(const SignalState& state, const Road& road);

private:

  TrajectoryGenerator trjGenerator;

  Predictor predictor;
  BehaviorPlanner behavior;

  int currentLane;
  int targetLane;
  double referenceVelocity;
};

#endif //__PATHPLANNER_H_

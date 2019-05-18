#ifndef _PATHPLANNER_H_
#define _PATHPLANNER_H_

#include "TrajectoryGenerator.h"
#include "Predictor.h"

#include <vector>
#include <array>

using namespace std;

class PathPlanner {
public:

  PathPlanner()
    : currentLane(0)
    , targetLane(0)
    , referenceVelocity(49.5)
    , currentType(GEN_SIMPLE)
  {}

  array<vector<double>, 2> plan(const SignalState& state);

private:
  enum GeneratorType {
    GEN_SPLINE,
    GEN_JMT,
    GEN_SIMPLE
  };

  TrajectoryGenerator trjGenerator;
  Predictor predictor;

  int currentLane;
  int targetLane;
  double referenceVelocity;
  GeneratorType currentType;
};

#endif //__PATHPLANNER_H_

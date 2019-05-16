#ifndef _PATHPLANNER_H_
#define _PATHPLANNER_H_

#include "TrajectoryGenerator.h"
#include "Predictor.h"

#include <vector>
#include <array>

using namespace std;

class PathPlanner {
public:
  enum LocalizationIndex {
    LOCAL_CAR_X = 0,
    LOCAL_CAR_Y,
    LOCAL_CAR_S,
    LOCAL_CAR_D,
    LOCAL_CAR_YAW,
    LOCAL_CAR_SPEED
  };

  enum EndPathIndex {
    END_PATH_S = 0,
    END_PATH_D
  };

  enum MapWaypointsIndex {
    MWP_X,
    MWP_Y,
    MWP_S,
    MWP_DX,
    MWP_DY
  };

  PathPlanner()
    : currentLane(0)
    , referenceVelocity(49.5)
    , currentType(GEN_SIMPLE)
  {}

  array<vector<double>, 2> plan(const array<double, 6>& localizationData,
      const vector<double>& prev_path_x, const vector<double>& prev_path_y,
      const array<double, 2>& end_path_Frenet,
      const vector<vector<double>>& sensor_fusion,
      const array<vector<double>, 5>& map_waypoints);

private:
  enum GeneratorType {
    GEN_SPLINE,
    GEN_JMT,
    GEN_SIMPLE
  };

  TrajectoryGenerator trjGenerator;
  Predictor predictor;

  int currentLane;
  double referenceVelocity;
  GeneratorType currentType;
};

#endif //__PATHPLANNER_H_

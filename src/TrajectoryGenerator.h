#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <array>
#include "spline.h"
//#include "helpers.h"

using namespace std;

class TrajectoryGenerator {
public:

  TrajectoryGenerator() {}

  array<vector<double>, 2> generateJMT();
  array<vector<double>, 2> generateSpline();

  array<vector<double>, 2> generateSimple(int lane,
    double refVel,
    const array<double, 6>& localizationData,
    const array<vector<double>, 5>& map_waypoints);
};

#endif // _TRAJECTORY_H_

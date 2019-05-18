#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <array>
#include "spline.h"
#include "helpers.h"

using namespace std;

struct SignalState;

class TrajectoryGenerator {
public:

  TrajectoryGenerator() {}

  array<vector<double>, 2> generateJMT(const int& lane, const double& ref_velocity, const SignalState& state);
  array<vector<double>, 2> generateSpline(const int& lane, const double& ref_velocity, const SignalState& state);
  array<vector<double>, 2> generateSimple(const int& lane, const double& ref_velocity, const SignalState& state);
};

#endif // _TRAJECTORY_H_

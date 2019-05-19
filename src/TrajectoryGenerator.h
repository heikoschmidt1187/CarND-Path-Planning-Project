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

  TrajectoryGenerator()
    : currentBaseVelocity(0.)
  {}

  array<vector<double>, 2> generateJMT(const int& lane, const double& ref_velocity, const SignalState& state);
  array<vector<double>, 2> generateSpline(const int& lane, const double& ref_velocity, const SignalState& state);
  array<vector<double>, 2> generateSimple(const int& lane, const double& ref_velocity, const SignalState& state);

private:
  double currentBaseVelocity;

  void updateCurrentBaseVelocity(double ref_velocity);
};

#endif // _TRAJECTORY_H_

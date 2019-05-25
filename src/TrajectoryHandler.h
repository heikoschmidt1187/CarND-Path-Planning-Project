#ifndef TRAJECTORYHANDLER_H_
#define TRAJECTORYHANDLER_H_

#include "Eigen-3.3/Eigen/Dense"
#include <vector>

#include "Car.h"

class TrajectoryHandler {
public:
  // generates a trajectory
  Car::Trajectory GenerateTrajectory(const Car::State& start_s, const Car::State& start_d,
    const Car::State& end_s, const Car::State& end_d, double T);

  // get the position from a given JMT
  static double getJmtPos(const Eigen::VectorXd& coeffs, const double t);

private:
  Eigen::VectorXd getJMT(const Car::State& start, const Car::State& end, const double T);
};

#endif /* TRAJECTORYHANDLER_H_ */

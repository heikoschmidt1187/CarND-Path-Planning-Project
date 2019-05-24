#ifndef TRAJECTORYGENERATOR_H_
#define TRAJECTORYGENERATOR_H_

#include "Eigen-3.3/Eigen/Dense"
#include <vector>

#include "Car.h"

namespace Trajectory {

class JMT {
public:
  JMT(const Car::State& start, const Car::State& end, const double T);
  double get(const double t);

private:
  Eigen::VectorXd c;
};
}

#endif /* TRAJECTORYGENERATOR_H_ */

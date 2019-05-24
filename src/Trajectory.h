#ifndef TRAJECTORYGENERATOR_H_
#define TRAJECTORYGENERATOR_H_

#include "Eigen-3.3/Eigen/Dense"
#include <vector>

#include "helpers.h"

namespace Trajectory {

class JMT {
public:
  JMT(const Helpers::VehicleState& start, const Helpers::VehicleState& end, const double T);
  double get(const double t);

private:
  Eigen::VectorXd c;
};
}

#endif /* TRAJECTORYGENERATOR_H_ */

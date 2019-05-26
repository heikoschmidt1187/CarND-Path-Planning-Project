#include <cmath>

#include "TrajectoryHandler.h"

Eigen::VectorXd TrajectoryHandler::getJMT(const Car::State& start, const Car::State& end, const double T)
{
  // pre-calculate powers of T to save time
  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;

  // solve: Ax = b
  // coefficiency matrix A
  Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
  A <<  T3,       T4,       T5,
        3 * T2,   4 * T3,   5 * T4,
        6 * T,    12 * T2,  20 * T3;

  // right side vector b
  Eigen::VectorXd b = Eigen::VectorXd(3);
  b <<  end.position - (start.position + start.velocity * T + 0.5 * start.acceleration * T2),
        end.velocity - (start.velocity + start.acceleration * T),
        end.acceleration - start.acceleration;

  Eigen::VectorXd x = Eigen::VectorXd(3);
  x = A.inverse() * b;

  // coefficents after JMT calculation
  Eigen::VectorXd ret = Eigen::VectorXd(6);
  ret <<  start.position, start.velocity, start.acceleration, x[0], x[1], x[2];

  return ret;
}

double TrajectoryHandler::getJmtVals(const Eigen::VectorXd& coeffs, const double t)
{
  // pre-calculate powers of t to save time
  const double t2 = t * t;
  const double t3 = t2 * t;

  Eigen::VectorXd T;

  if(coeffs.rows() == 6) {
    
    const double t4 = t3 * t;
    const double t5 = t4 * t;

    T = Eigen::VectorXd(6);
    T << 1.0, t, t2, t3, t4, t5;
  } else if(coeffs.rows() == 5) {

    const double t4 = t3 * t;

    T = Eigen::VectorXd(5);
    T << 1.0, t, t2, t3, t4;
  } else {
    T = Eigen::VectorXd(4);
    T << 1.0, t, t2, t3;
  }

  return T.transpose() * coeffs;
}

Car::Trajectory TrajectoryHandler::GenerateTrajectory(const Car::State& start_s, const Car::State& start_d,
    const Car::State& end_s, const Car::State& end_d, double T)
{
  // for now, just calculate the JMTs and return the coefficients
  Eigen::VectorXd coeffs_s = getJMT(start_s, end_s, T);
  Eigen::VectorXd coeffs_d = getJMT(start_d, end_d, T);

  Eigen::VectorXd coeffs_s_dot = Eigen::VectorXd(5);
  Eigen::VectorXd coeffs_s_dot_dot = Eigen::VectorXd(4);
  Eigen::VectorXd coeffs_d_dot = Eigen::VectorXd(5);
  Eigen::VectorXd coeffs_d_dot_dot = Eigen::VectorXd(4);

  // derivate once for *_dot
  for(int i = 1; i < 6; ++i) {
    coeffs_s_dot[i - 1] = static_cast<double>(i) * coeffs_s[i];
    coeffs_d_dot[i - 1] = static_cast<double>(i) * coeffs_d[i];
  }

  // derivate again for *_dot_dot
  for(int i = 1; i < 5; ++i) {
    coeffs_s_dot_dot[i - 1] = static_cast<double>(i) * coeffs_s_dot[i];
    coeffs_d_dot_dot[i - 1] = static_cast<double>(i) * coeffs_d_dot[i];
  }

  return {coeffs_s, coeffs_s_dot, coeffs_s_dot_dot, coeffs_d, coeffs_d_dot, coeffs_d_dot_dot, T};
}

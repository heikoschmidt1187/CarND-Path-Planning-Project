#ifndef CAR_H_
#define CAR_H_

#include "Eigen-3.3/Eigen/Dense"
#include <vector>

class Car {
public:
  struct Trajectory {
    Eigen::VectorXd c_s;    // trajectory s coefficients
    Eigen::VectorXd c_d;    // trajectory d coefficients
    double t;               // trajectory time
  };

  struct State {
    double position;        // car position in s or d
    double velocity;        // car velocity in s or d
    double acceleration;    // car acceleration in s or d
  };

public:
  // C'tors for Car class
  Car();
  Car(int carId);
  Car(int carId, double S, double D, double Speed);

  // updates the car's state
  void update(double S, double D, double Speed);

  // getters
  double getId() const { return id; }
  double getS() const { return s; }
  double getD() const { return d; }
  double getSpeed() const { return speed; }
  int getLane() const;


private:
  int id;         // unique vehicle id
  double s;       // car's s position in m
  double d;       // car's d position in m
  double speed;   // car's speed in m/s
};

#endif /* CAR_H_ */

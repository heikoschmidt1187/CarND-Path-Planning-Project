#include <iostream>

#include "Parameter.h"
#include "Car.h"

Car::Car()
  : id(-1), lane(-1), s(0.), d(0.), speed(0.)
{}

Car::Car(int carId)
  : id(carId), lane(-1), s(0.), d(0.), speed(0.)
{}

Car::Car(int carId, double S, double D, double Speed)
  : id(carId), s(S), d(D), speed(Speed)
{
  lane = Car::calcLane(d);
}

void Car::update(double S, double D, double Speed)
{
  s = S;
  d = D;
  speed = Speed;

  lane = Car::calcLane(d);
}

int Car::calcLane(const double d)
{
  // calculate new lane
  for(unsigned i = 0; i < Parameter::k_lane_count; ++i) {
    if(d < (Parameter::k_lane_width * (i + 1))) {
      return i;
    }
  }

  return -1;
}

#include "Car.h"

Car::Car(int id)
{
  this->id = id;
}

void Car::setPosition(const double s, const double d)
{
  this->s = s;
  this->d = d;
}

void Car::setSpeed(const double velocity)
{
  v = velocity;
}

#ifndef CAR_H_
#define CAR_H_

class Car {
public:
  Car(const int id);
  void setPosition(const double s, const double d);
  void setSpeed(const double velocity);

private:
  int id;
  double s;
  double d;
  double v;
};


#endif /* CAR_H_ */

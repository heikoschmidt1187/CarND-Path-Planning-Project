#ifndef CAR_H_
#define CAR_H_

class Car {
public:
  struct State {
    double position;
    double velocity;
    double acceleration;
  };

public:
  Car()
    : id(-1), s(0.), d(0.), speed(0.)
  {}

  Car(int carId)
    : id(carId), s(0.), d(0.), speed(0.)
  {}

  Car(int carId, double S, double D, double Speed)
    : id(carId), s(S), d(D), speed(Speed)
  {}

  void update(double S, double D, double Speed)
  {
    s = S;
    d = D;
    speed = Speed;
  }

  double getId() const { return id; }
  double getS() const { return s; }
  double getD() const { return d; }
  double getSpeed() const { return speed; }

private:
  int id;
  double s;
  double d;
  double speed;
};

#endif /* CAR_H_ */

#ifndef _PREDICTOR_H_
#define _PREDICTOR_H_

#include "helpers.h"

class Predictor {
public:
  Predictor()
  {}

  void predict(const SignalState& state, const Road& road);
};

#endif // _PREDICTOR_H_

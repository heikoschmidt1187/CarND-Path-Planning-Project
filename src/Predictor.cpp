#include "Predictor.h"

void Predictor::predict(const SignalState& state, const Road& road)
{
  // first simple approach - check for car in own lane for speed up/slow down
  double dist_to_car_in_lane = 999.;
  for(const auto& s : state.sensor_fusion) {

  }
}

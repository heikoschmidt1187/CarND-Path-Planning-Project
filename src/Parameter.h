#ifndef PARAMETER_H_
#define PARAMETER_H_

class Parameter {
public:
  // road
  static constexpr float k_lane_width = 4.0;          // m
  static constexpr int k_lane_count = 3;              // number of lanes

  // speed
  static constexpr float k_speed_limit = 22.2;        // m/s
  static constexpr float k_speed_buffer = 2.;         // m/s

  // prediction and planning
  static constexpr float k_prediction_time = 2.;      // s
  static constexpr int k_min_lane_change_cycles = 5;  // minimum cylces for LC to avoid too much hopping
};

#endif /* PARAMETER_H_ */

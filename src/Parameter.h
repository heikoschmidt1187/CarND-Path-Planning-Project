#ifndef PARAMETER_H_
#define PARAMETER_H_

class Parameter {
public:
  // road
  static constexpr float k_lane_width = 4.0;          // m
  static constexpr int k_lane_count = 3;              // number of lanes

  // map parameters
  static constexpr float k_max_s = 6945.554;           // maximum s value before track end
  static constexpr float k_max_interpol_window = 300.; // interpolation window around current s position

  // speed
  static constexpr float k_speed_limit = 22.2;        // m/s
  static constexpr float k_speed_buffer = 2.;         // m/s

  // prediction and planning
  static constexpr float k_prediction_time = 2.;      // s
  static constexpr int k_min_lane_change_cycles = 5;  // minimum cylces for LC to avoid too much hopping
  static constexpr double k_gap_buffer_time = 1.;     // gap buffer when following a car

  // costs
  static constexpr float k_lane_number_weight = 1.;   // weight for lane number cost for lane speed calculation
  static constexpr float k_lane_speed_weight = 5.;    // weight for lane speed cost for lane cost calculation
};

#endif /* PARAMETER_H_ */

#include <cmath>

#include "MapHelper.h"

MapHelper::MapHelper(const std::vector<double>& waypoints_s,
            const std::vector<double>& waypoints_x,
            const std::vector<double>& waypoints_y)
{
  for(size_t i = 0; i < waypoints_s.size(); ++i) {
    waypoints.insert(std::make_pair(waypoints_s.at(i), Waypoint{i,
      waypoints_x.at(i), waypoints_y.at(i)}));
  }
}

std::vector<double> MapHelper::getXY(double base_s, double s, double d)
{
  // first set the interpolation region spline around the current base_s
  setCurrentSplinePoints(base_s);

  // calculate the map coordinates
  double x = waypoint_spline_x(s);
  double y = waypoint_spline_y(s);

  // the value is representing the value for d = 0, so we need to calculate the correct
  // d value

  // get the normal vector for x and y to the spline point by using the first derivative
  // at the desired s point
  double x_normal = waypoint_spline_y.first_derivative(s);
  double y_normal = -waypoint_spline_x.first_derivative(s);

  // calculate the normalized distance from the point to the desired d
  double dist = std::sqrt(pow(d, 2) - pow(x_normal, 2) - pow(y_normal, 2));

  return {x + dist * x_normal, y + dist * y_normal};
}

void MapHelper::setCurrentSplinePoints(double s)
{
  // to get a smooth trajectory, the waypoint data needs to be interpolated between
  // consecutive points - as this is only needed locally, a window arount the current
  // s value is defined where the spline is generated

  // find the lower and upper s position
  double lower_s = s - Parameter::k_max_interpol_window;
  double upper_s = s + Parameter::k_max_interpol_window;

  // special attention needs to be payed on the overflow between consecutive rounds
  // so helpers are needed

  // get the current round from s
  double lower_round = std::floor(lower_s / Parameter::k_max_s);
  double upper_round = std::floor(upper_s / Parameter::k_max_s);

  // we also need the current s position relative to the track start for each round
  double lower_rel_s = std::fmod(lower_s, Parameter::k_max_s);
  double upper_rel_s = std::fmod(upper_s, Parameter::k_max_s);

  // the lower relative s may be negative if we are near the track begin, so shift one round
  if(lower_rel_s < 0) {
    lower_rel_s += Parameter::k_max_s;
  }

  // we now iterate from the lowest relative s (in detail: the next s from map data)
  // to the upper relative s and create the points for the spline calculation
  std::vector<double> wp_s, wp_x, wp_y;
  std::vector<size_t> wp_id;

  auto it = waypoints.upper_bound(lower_rel_s);

  // take care for the special case that the lower relative s is before the upper relative
  // s in the sense of track rounds
  if(lower_round != upper_round) {
    // push the values from the previous round from the map data to the vectors multiplied
    // by the current round number
    while(it != waypoints.end()) {
      wp_id.push_back(it->second.id);
      wp_s.push_back(it->first + Parameter::k_max_s * lower_round);
      wp_x.push_back(it->second.x);
      wp_y.push_back(it->second.y);
      ++it;
    }

    // we pushed the "old" round, set the iterator to the first element of the new round
    it = waypoints.begin();
  }

  // iterate till the end
  while((it != waypoints.end()) && (it->first < upper_rel_s)) {
      // push the values from the previous round from the map data to the vectors multiplied
      // by the current round number
      wp_id.push_back(it->second.id);
      wp_s.push_back(it->first + Parameter::k_max_s * upper_round);
      wp_x.push_back(it->second.x);
      wp_y.push_back(it->second.y);
      ++it;
  }

  // set the spline points for further calculation - but avoid multiple calculation on same data
  if(waypoint_ids != wp_id) {
    waypoint_ids = wp_id;
    waypoint_spline_x.set_points(wp_s, wp_x);
    waypoint_spline_y.set_points(wp_s, wp_y);
  }
}

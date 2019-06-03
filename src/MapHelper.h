#ifndef  MAPHELPER_H_
#define MAPHELPER_H_

#include <vector>
#include <map>

#include "spline.h"
#include "Parameter.h"

// structure representing a map waypoint
struct Waypoint {
  size_t id;
  double x;
  double y;
};

class MapHelper {
public:
  MapHelper(const std::vector<double>& waypoints_s,
            const std::vector<double>& waypoints_x,
            const std::vector<double>& waypoints_y);

  std::vector<double> getXY(double base_s, double s, double d);

private:
  // the waypoint's logical connection is represented via a map
  std::map<double, Waypoint> waypoints;
  std::vector<size_t> waypoint_ids;

  tk::spline waypoint_spline_x;
  tk::spline waypoint_spline_y;

  void setCurrentSplinePoints(double s);
};

#endif /* MAPHELPER_H_ */

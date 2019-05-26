#include "Parameter.h"
#include "PathPlanner.h"
#include "helpers.h"

PathPlanner::PathPlanner(const std::vector<double>& waypoints_s,
  const std::vector<double>& waypoints_x,
  const std::vector<double> waypoints_y,
  const std::vector<double> waypoints_dx,
  const std::vector<double> waypoints_dy)
{
  // push points to spline for later smooting lane
  waypoint_spline_x.set_points(waypoints_s, waypoints_x);
  waypoint_spline_y.set_points(waypoints_s, waypoints_y);
  waypoint_spline_dx.set_points(waypoints_s, waypoints_dx);
  waypoint_spline_dy.set_points(waypoints_s, waypoints_dy);
}

std::vector<std::vector<double>> PathPlanner::update(const Car& ego,
    const std::vector<double>& previous_path_x,
    const std::vector<double>& previous_path_y,
    const std::vector<Car>& other_cars)
{
  // output vectors
  std::vector<double> next_x;
  std::vector<double> next_y;

  // check current lane from state and set if needed
  if(current_state.lane < 0) {
    current_state.lane = ego.getLane();
  }

  // update the previous s and d values according to the points not driven by
  // the simulator in cycle
  previous_path_s.erase(previous_path_s.begin(), previous_path_s.begin()
    + previous_path_s.size() - previous_path_x.size());

  previous_path_d.erase(previous_path_d.begin(), previous_path_d.begin()
    + previous_path_d.size() - previous_path_y.size());

  // first attempt: let the car drive in lane while maintaining speed

  double s, s_vel, s_acc;
  double d, d_vel, d_acc;

  // reuse previous path and use last s/d as base for new trajectory calculation
  if(previous_path_x.size() > 0) {

    s = previous_path_s.back().position;
    s_vel = previous_path_s.back().velocity;
    s_acc = previous_path_s.back().acceleration;

    d = previous_path_d.back().position;
    d_vel = previous_path_d.back().velocity;
    d_acc = previous_path_d.back().acceleration;
  }
  else {
    previous_path_s.clear();
    previous_path_d.clear();

    s = ego.getS();
    s_vel = ego.getSpeed();
    s_acc = 0;

    d = ego.getD();
    d_vel = 0;
    d_acc = 0;
  }
  // clear debug output
  /*
#if defined(_WIN32) || defined(_WIN64)
  system("cls")
#else
  system("clear");
#endif
*/

  auto future = behavior_handler.plan({s, s_vel, s_acc}, {d, d_vel, d_acc}, other_cars);

  if(previous_path_x.size() > 0) {
    for(size_t i = 0; i < previous_path_x.size(); ++i) {
      next_x.push_back(previous_path_x.at(i));
      next_y.push_back(previous_path_y.at(i));
    }
  }

  double time = (50 - previous_path_x.size()) * 0.02;

  // adapt speed change to avoid excessive jerk or acceleration
  if(s_vel > future.speed) {
    future.speed = std::max(future.speed, s_vel - time);
  } else if(s_vel < future.speed) {
    future.speed = std::min(future.speed, s_vel + time);
  }

  std::cout << "=== Trajectory calculation ===" << std::endl;
  std::cout << "Speed: " << future.speed << ", Lane: " << future.lane << std::endl;

  std::cout << "Start: 0 * " << s_vel << " * " << s_acc << std::endl;
  std::cout << "Start: " << d << " * " << d_vel << " * " << d_acc << std::endl;

  std::cout << "End: " << time * future.speed << " * " << future.speed << " * 0" << std::endl;
  std::cout << "End: " << 2. + future.lane * 4. << " * 0 * 0" << std::endl;

  auto trajectory = trajectory_handler.GenerateTrajectory(
    {0, s_vel, s_acc},
    {d, d_vel, d_acc},
    {time * future.speed, future.speed, 0},
    {2. + future.lane * 4., 0, 0},
    time);


  // calculate the states for each cycle - for this we need the derivatives of the coefficients
  std::cout << "**** new points ***" << std::endl;

  for(int i = 1; i <= (50 - previous_path_x.size()); ++i) {

    double new_s = TrajectoryHandler::getJmtVals(trajectory.c_s, i * 0.02) + s;
    double new_s_dot = TrajectoryHandler::getJmtVals(trajectory.c_s_dot, i * 0.02);
    double new_s_dot_dot = TrajectoryHandler::getJmtVals(trajectory.c_s_dot_dot, i * 0.02);

    double new_d = TrajectoryHandler::getJmtVals(trajectory.c_d, i * 0.02);
    double new_d_dot = TrajectoryHandler::getJmtVals(trajectory.c_d_dot, i * 0.02);
    double new_d_dot_dot = TrajectoryHandler::getJmtVals(trajectory.c_d_dot_dot, i * 0.02);

    //vector<double> xy = Helpers::getXY(s, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    previous_path_s.push_back({new_s, new_s_dot, new_s_dot_dot});
    previous_path_d.push_back({new_d, new_d_dot, new_d_dot_dot});
    std::vector<double> xy = Helpers::getXY(new_s, new_d, waypoint_spline_x, waypoint_spline_y, waypoint_spline_dx, waypoint_spline_dy);

    next_x.push_back(xy.at(0));
    next_y.push_back(xy.at(1));

    std::cout << new_d << " * ";
  }
  std::cout << "\n\n";


  return {next_x, next_y};
}

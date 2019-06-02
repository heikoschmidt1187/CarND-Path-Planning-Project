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
    const std::vector<Car>& other_cars,
    const std::vector<double>& map_waypoints_s,
    const std::vector<double>& map_waypoints_x,
    const std::vector<double>& map_waypoints_y)
{
  // output vectors
  std::vector<double> next_x;
  std::vector<double> next_y;

  // check current lane from state and set if needed
  if(current_state.lane < 0) {
    current_state.lane = ego.getLane();
  }

  if((previous_path_x.size() > 0) && (previous_path_x.size() == previous_path_s.size())) {
    // in this case, the simulator did not consume any of the generated points (usually not the case,
    // did occur though while testing different states)
    for(size_t i = 0; i < previous_path_x.size(); ++i) {
      next_x.push_back(previous_path_x.at(i));
      next_y.push_back(previous_path_y.at(i));
    }
  } else {
    // update the previous s and d values according to the points not driven by
    // the simulator in cycle
    previous_path_s.erase(previous_path_s.begin(), previous_path_s.begin()
      + previous_path_s.size() - previous_path_x.size());

    previous_path_d.erase(previous_path_d.begin(), previous_path_d.begin()
      + previous_path_d.size() - previous_path_x.size());

    // clear debug output
  #if defined(_WIN32) || defined(_WIN64)
    system("cls")
  #else
    system("clear");
  #endif

    // reuse previous path and use last s/d as base for new trajectory calculation
    double farest_s = 0;

    // first attempt: let the car drive in lane while maintaining speed
    Car::State begin_s;
    Car::State begin_d;

    if(previous_path_s.size() > 0) {

      begin_s = previous_path_s.back();
      begin_s.position = 0.;
      begin_d = previous_path_d.back();

      farest_s = previous_path_s.back().position;
    }
    else {
      previous_path_s.clear();
      previous_path_d.clear();

      begin_s = {0., 0., 0.};
      begin_d = {ego.getD(), 0, 0};

      farest_s = ego.getS();
    }


    auto future = behavior_handler.plan(ego, {farest_s, begin_s.velocity, begin_s.acceleration},
       previous_path_s.size() * 0.02,
       {begin_d.position, begin_d.velocity, begin_d.acceleration}, other_cars);

    // depending on if fast reaction is needed, we do not use all the previous path
    if(future.need_fast_reaction == false) {

      // keep the previous planned points
      if(previous_path_x.size() > 0) {
        for(size_t i = 0; i < previous_path_x.size(); ++i) {
          next_x.push_back(previous_path_x.at(i));
          next_y.push_back(previous_path_y.at(i));
        }
      }
    } else {

      // fast reaction is needed, only keep 5 points to compensate for simulator delay
      std::cout << "Fast reaction..." << std::endl;

      // clean the state to only use the next 5 points
      if(previous_path_x.size() > 5) {
        previous_path_s.erase(previous_path_s.begin() + 5, previous_path_s.end());
        previous_path_d.erase(previous_path_d.begin() + 5, previous_path_d.end());
      }

      // only use 5 previous points...
      for(size_t i = 0; i < 5; ++i) {
          next_x.push_back(previous_path_x.at(i));
          next_y.push_back(previous_path_y.at(i));
      }

      // TODO: refactor
      if(previous_path_s.size() > 0) {

        begin_s = previous_path_s.back();
        begin_s.position = 0.;
        begin_d = previous_path_d.back();

        farest_s = previous_path_s.back().position;
      }
      else {
        previous_path_s.clear();
        previous_path_d.clear();

        begin_s = {0., 0., 0.};
        begin_d = {ego.getD(), 0, 0};

        farest_s = ego.getS();
      }
    }


    int missing_ponts = 50 - previous_path_s.size();

    // 50 points ==> 1 sec is used for trajectory generation, but we plan wide
    double time = Parameter::k_prediction_time + static_cast<double>(missing_ponts) * 0.02;

    // adapt speed change to avoid excessive jerk or acceleration (2 m/s/s)
    // TODO: behavior can tell the speedup/slowdown rate based on other traffic --> use this here
    double target_speed = 0.;

    if(future.speed > begin_s.velocity)
      target_speed = std::min(begin_s.velocity + time, future.speed);
    else if(future.speed < begin_s.velocity)
      target_speed = std::max(begin_s.velocity - time, future.speed);
    else
      target_speed = future.speed;

    std::cout << "=== Trajectory calculation ===" << std::endl;
    std::cout << "Planning for " << time << " seconds" << std::endl;
    std::cout << "Target Speed: " << target_speed << ", Lane: " << future.lane << std::endl;

    std::cout << "Start: " << begin_s.position << " - " << begin_s.velocity << " - " << begin_s.acceleration << std::endl;
    std::cout << "Start: " << begin_d.position << " - " << begin_d.velocity << " - " << begin_d.acceleration << std::endl;

    std::cout << "End: " << time * target_speed << " * " << target_speed << " * 0" << std::endl;
    std::cout << "End: " << 2. + future.lane * 4. << " * 0 * 0" << std::endl;

    std::cout << "Missing Points: " << missing_ponts << std::endl;
    std::cout << "Planning Time: " << time << std::endl;

    auto trajectory = trajectory_handler.GenerateTrajectory(
      begin_s,
      begin_d,
      {time * target_speed, target_speed, 0},
      {2. + future.lane * 4., 0, 0},
      time);

    // calculate the states for each cycle - for this we need the derivatives of the coefficients
    for(int i = 1; i <= (missing_ponts + 1); ++i) {

      double new_s = TrajectoryHandler::getJmtVals(trajectory.c_s, i * 0.02) + farest_s;
      double new_s_dot = TrajectoryHandler::getJmtVals(trajectory.c_s_dot, i * 0.02);
      double new_s_dot_dot = TrajectoryHandler::getJmtVals(trajectory.c_s_dot_dot, i * 0.02);

      double new_d = TrajectoryHandler::getJmtVals(trajectory.c_d, i * 0.02);
      double new_d_dot = TrajectoryHandler::getJmtVals(trajectory.c_d_dot, i * 0.02);
      double new_d_dot_dot = TrajectoryHandler::getJmtVals(trajectory.c_d_dot_dot, i * 0.02);

      previous_path_s.push_back({new_s, new_s_dot, new_s_dot_dot});
      previous_path_d.push_back({new_d, new_d_dot, new_d_dot_dot});

      std::vector<double> xy = Helpers::getXY(new_s, new_d, waypoint_spline_x, waypoint_spline_y, waypoint_spline_dx, waypoint_spline_dy);

      next_x.push_back(xy.at(0));
      next_y.push_back(xy.at(1));

    }
  }

  return {next_x, next_y};
}

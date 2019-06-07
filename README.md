# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

# Writeup for the project
## Rubric points

The following rubric points were met to fullfil the project requirements:

### The code compiles correctly.

The code compiles successful without any errors. With the current compile options, there are warnings for the Spline library that fields are used that have an anonymous namespace. While this may be corrected in the future, it doesn't have an impact on the implementation and can be ignored safely.

### The car is able to drive at least 4.32 miles without incident.

During my tests the car was able to drive several rounds on the track without any incidents most of the time. Besides that, there have been two rare cases where there were sporadic incidents where 1) the car was "out of lane" according to the simulator - from an optical perspective this wasn't the case as the car was in the middle of the rightmost lane - and 2) that there was a contact with another car that changed into our own lane very short before us. The last point was fixed with more defensive parameters and a better prediction of the position.

As both points did not occur in the following tests and I wasn't able to reproduce them, I need to rely on the fact that the car can drive multiple rounds without incident.

### The car drives according to the speed limit.

The parameters have been chosen carefully to avoid speeding of the car at any cost. To have some space for maneuvers where fast reaction is needed, the target speed is at 45mph - 5mph below the speed limit. This leads to a smooth driving behavior in normal traffic.

If there are slower cars in front and there's no lane change possible in a safe manner, the car slows down and maintains a safe distance to the traffic in front to be able to react in case anything unforseen happens.

### Max Acceleration and Jerk are not Exceeded.

The implementation uses Jerk Minimizing Trajectories (JMTs) to avoid excessive jerk. The maximum acceleration/deceleration is limited due to the time for a lane change (2s) and the maximum speed change in s direction (1m/s^2) in normal driving and 2m/s^2 when fast reaction is needed (fast slow down).

### Car does not have collisions.

The car is constantly monitoring the traffic around and checks for speed and lane change possibility. More on that in the implementation description below.

### The car stays in its lane, except for the time between changing lanes.

The implementation makes the car stay on the right side of the road. It has an affinity to the middle lane to have most of the time more possibilities to react (stay, change left, change right). If the car goes to the left or right lane due to a lane change, it will change back to the middle as soon as it's safe and the lane has the best speed possibility.

The car only leaves a lane for a change - the target time for a complete lane change is 2s.

### The car is able to change lanes

As already written, the car is able to change lanes safely if the traffic in front is too slow. Most of the time it only changes lanes if this improves the speed for getting forward. Sometimes the changes are reverted because of the limited view into the future. This may be improved through better cost functions or delays for keep lane after lane changes.

## Implementation description (Model Documentation)

Finding a feasable path consists of different steps that are running in a cyclic manner. The steps have been encapsulated in different modules, like the following:

|   Step    |   Module    |
|-----------|-------------|
| Reading Map Data from file (once), getting data from simulator, send processed data to simulator   | main.cpp  |
| Update the current state of the cars on the track   | Car.h/cpp  |
| Plan the movement for the next 2 seconds and return the Trajectory for the next 1 second   | PathPlanner.h/cpp |
|  To generate a trajectory, the current state has to be used to predict the future and decide what to do (keep lane, change lanes, speed up, slow down, ...)  | BehaviorHandler.h/cpp  |
| After decision, a safe trajectory needs to be generated  | TrajectoryHandler.h/cpp  |
| To get drivable points, the points need to be converted into map coordinates and the space between given map waypoints need to be interpolated for a smooth movement   | MapHelper.h/cpp, spline.h, helpers.h  |

In the following sections I will describe each of the above mentioned steps in detail and link it with the actual code.

### main.cpp

The code in main.cpp was provided by Udacity and implements the initial reading of the map data from a csv file and the cyclic communication with the simulator.

The first modification was made after the initial reading *lines 60 to 64*. Here an object of the class *PathPlanner* is created and updated with the readout data - more on this below. The path planner is the core of my implementation and combines all necessary steps to make the car drive on it's own by returning points to be sent to the simulator.

Also an object of the class *Car* is created to hold the current state of the *ego* car.

The next modification has been done in the cyclic communication part with the simulator. You can find them *lines 114 to 132*.

First, the ego state is updated with the current Frenet coordinates and speed of the own car. Next, the sensor fusion data is read out and converted to the *Car* class' format for further usage by the path planner object.

Note here that each car's speed is calculated using the x- and y-speed component vectors:

```C++
  Car other(static_cast<int>(s[0]));

  double speed = sqrt(pow(s[3], 2) + pow(s[4], 2));
  other.update(s[5], s[6], speed);

  other_cars.push_back(other);
```

All cars are pushed to the *other_cars* vector, which is also used by the path planner object.

As a last point, the *update* function of the path planner is called to start the processing and retrieve the x- and y-coordinates of the points to be driven by the simulator:

```C++
  // plan!
  auto xy = planner.update(ego,
    previous_path_x, previous_path_y, other_cars, map_waypoints_s, map_waypoints_x, map_waypoints_y);
```

### The path planner class (PathPlanner.h/PathPlanner.cpp)

The path planner is the core of the whole algorithm. It combines the functional steps for prediction, planning the behavior and generating a trajectory.

First of all, there's a special point in the constructor of the path planner at *PathPlanner.cpp line 10* - here an object of the MapHelper class is created.

While implementing the path planner, I soon faced problems with converting the trajectory points to map coordinates. Almost always the acceleration and jerk went beyond the target values, the speed was too high even if the spacing between the points was ok. In the Q&A videos the spline.h library was used to smoothen the path and new points were generated using the two last points of the already existing path.

As I wanted to use the JMT knowledge from the lessions, I decided not to use the approach from the Q&A video. But to get a smooth path, it was obvious that I need to interpolate the points between the given waypoints and use my own function to transform Frenet coordinates to map coordinates. That's what the MapHelper class is used for.

The *MapHelper* class has two important functions: *setCurrentSplinePoints* and *getXY*.

You can find the setCurrentSplinePoints function in *MapHelper.cpp lines 39 to 105*. The function uses the map waypoints around a given s-coordinate to setup the points of the spline class locally in a search window around that s-coordinate. The point here is that - as the JMT points are calculated as s-coordinated in a consecutive manner over rounds - the waypoints are transfered to match the position in the current track round. This is done by checking where on the track ("inside" a track round) and which round (n-th round starting by 0) the s is located in. If the search window overlaps two rounds, the values from the old and the new round are fitted.

So with this, even s-values > than the 6945.554m (defined in Parameter.h, evaluated as value where the simulator overflows the s value to 0 at the track beginning) can be converted smoothely, so no problems occur on driving over the zero track point from the map data.

The getXY function in *MapHelper.cpp lines 15 to 36* than uses the current spline objects to get the map coordinates from Frenet coordinates around the base s point. Note here, that first a base x- and y-coordinate is evaluated by the splines using the Frenet s. Then, for the resulting spline point the first derivatives are used to get the directions and distance to the desired Frenet d-coordinate. The normals are then used to calculate the final x and y map coordinate. The first_derivative function in the spline class is not present in the original library and was implemented for the usage in this project.

Back to the path planner. The function that triggers the cyclic algorithm is called *update* and can be found in *PathPlanner.cpp lines 14 to 194*. It works the following way:

In the *lines 23 to 82* the function checks how much path points are present after the last simulator cycle. This is the difference between the last given path and the consumed points by the car in the cycle. This value is then used to remove the same amount of points from the vectors *previous_path_s* and *previous_path_d* that represent the calculated trajectory path points:

```C++
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

[...]

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
```

The implementation needs to handle special cases where the previous path is empty or the simulator did not consume any points. Also, the current car state is calculated as this is used by the BehaviorPlanner object to plan the next steps. This is done with the following call:

```C++
  auto future = behavior_handler.plan(ego, {farest_s, begin_s.velocity, begin_s.acceleration},
     previous_path_s.size() * 0.02,
     {begin_d.position, begin_d.velocity, begin_d.acceleration}, other_cars);
```

The BehaviorPlanner is explained in the next section.

The result of the behavior planner is used in *PathPlanner.cpp lines 87 to 131* to decide how much points of the last path should be reused. In the "normal" driving mode, all points are reused and later extended to get a total number of points to have a trajectory of at lease 1sec length. If there is any special case where we need immediate reaction, like chaning lanes or slowing down fast because a car is getting into our lane near us, the *need_fast_reaction* variable is set and only up to five of the previous points are reused. Why not 0? Because there's a delay between sending the points and using them in the simulator, so the car does not drive smoothely. Five points lead to a good driving.

In *PathPlanner lines 134 to 161* the number of needed path points for the trajectory is calculated, and the time for the trajectory prediction for the JMT calculation is defined:

```C++
  int missing_ponts = 50 - previous_path_s.size();

  // 50 points ==> 1 sec is used for trajectory generation, but we plan wide
  double time = Parameter::k_prediction_time + static_cast<double>(missing_ponts) * 0.02;

  // adapt speed change to avoid excessive jerk or acceleration (1 m/s/s)
  // TODO: behavior can tell the speedup/slowdown rate based on other traffic --> use this here
  double target_speed = 0.;

  if(future.speed > begin_s.velocity)
    target_speed = std::min(begin_s.velocity + time, future.speed);
  else if(future.speed < begin_s.velocity)
    target_speed = std::max(begin_s.velocity - time, future.speed);
  else
    target_speed = future.speed;

[...]

```

Also, the speed change is limited to avoid too high acceleration values. This is done by increasing or decreasing the speed to the target speed from the behavior handler just about 1 m/s/s - which works fine in the cases that occur in the simulator. This may be further improved to react on unforseen situations to slow down or speed up with a higher velocity change rate.

After that, the TrajectoryHandler object is called to generate coefficients for a JMT:

```C++
  auto trajectory = trajectory_handler.GenerateTrajectory(
    begin_s,
    begin_d,
    {time * target_speed, target_speed, 0},
    {2. + future.lane * 4., 0, 0},
    time);
```

The TrajectoryHandler is described in one of the next sections.

And of course, the coefficients then are used to get the next points to fill the previous path up to 50 points (or 1 sec):

```C++
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

    //std::vector<double> xy = Helpers::getXY(new_s, new_d, waypoint_spline_x, waypoint_spline_y, waypoint_spline_dx, waypoint_spline_dy);
    std::vector<double> xy = map_helper.getXY(previous_path_s.front().position, new_s, new_d);

    next_x.push_back(xy.at(0));
    next_y.push_back(xy.at(1));

  }
}
```

### Planning the next steps (BehaviorHandler.h/BehaviorHandler.cpp)
<TODO>

### Getting a feasable JMT (TrajectoryHandler.h/TrajectoryHandler.cpp)
<TODO>

## Original Udacity project instructions README

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

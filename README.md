# CarND-Path-Planning-Project

Self-Driving Car Engineer Nanodegree Program

## Simulator

You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

## Goals

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### The map of the highway is in data/highway_map.txt

Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
    1. Also clone submodules `git submodule update --init`
2. Setup: `./install-ubuntu.sh`.
3. Build it: `./build.sh`.
4. Run it: `./run.sh`.

Here is the data provided from the Simulator to the C++ Program

### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates].

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using [http://kluge.in-chemnitz.de/opensource/spline/](http://kluge.in-chemnitz.de/opensource/spline/), the spline function is in a single hearder file is really easy to use.

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
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```sh
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

## Describe how path are generated

For generating path's, a `BehaviorPlanner` and a `TrajectoryPlanner` are used.
The `BehaviorPlanner` checks every 3 seconds if there is a lange change necessary or not.

The `BehaviorPlanner` always outputs the maximum velocity to drive, which lane and which vehicle to follow, if any.
He also includes the minimum safety time to other vehicles in his returning behavior object.

The `TrajectoryPlanner` uses the behavior object he gets as input and tries to fulfill the requested behavior.
He calculates the exact behavior.
He uses frenet coordinates to calculate the safety zone, when following another vehicle.
After the needed acceleration is calculated, the planner uses the cartesian coordinate system to create the exact trajectory.

All calculations use semantic units, such as Meter, Second, Radian, Degree.
This ensures, that the expected unit of the calculated values is the same as the formula creates.
To keep the performance untouched, this is done at compile-time.

### Changes in 2nd Submission

**Event-based logging and prefixes:** only write somthing to the log, if the state which you'd like to watch has changed. Also there are prefixes in front of each line, to idetify from which part of the software the change occured.

**Behavior planner maximum speed:** Behavior planner always asks for maximum speed, because the trajectory planner itself can decide, when to reduce speed, because of traffic or maximum acceleration.

**Behavior planner watch traffic during (prepare) lane change:** Behavior planner looks some distance behind the current frenet s coordinate in the new lane, to see if there is traffic and tries to slow down, if there is a car, to get behind this car.

**Behavior planner update time:** reduced the update time from 3 seconds to 800 milliseconds. This was able because of other fixes. Now the car changes lanes more often.

**Prediction of cars into the future:** Assume the cars will stay in the lane they are at the moment (no lane change) all the time. This can lead to collisions if any car does a lane change, but this is so rare, that the benefit is greater.

**Added some more GnuPlot debugging plots:** One to analyze the map and coordinate conversion. And another to analyze the trajectory planner.

**Fix some issues during coordinate conversion:** `next_way_point_index` did not return the next waypoint at each position around the track. In the function `normalize_around_zero` the define `M_2_PI` was used wrong. It was thought to be 2 times pi but actualy it was about 0.2 times pi and this introduced big issues in the coordinate conversion calculations. `M_2_PI` is now replaced with the correct value of `2 * M_PI`.

**Trajectory planner reuse previous points:** Increase the amount of reused points. This reduces the errors during lane change, which the simulator outputs (max acceleration and max velocity).

**Trajectory planner activly limits maximum speed:** The maximum speed is activly limited by the trajectory planner, during coordinate calculation.

**Trajecotry planner has a fixed lane change distance:** The trajectory planner does no more calculate the speed of the lane change based on the current speed. It now uses a fixed distance of 50 meters to change from one lane to another.

**Trajectory planner calculates the angle of the car:** The angle of the car in the current part of the spline is calculated, so the speed can be calculated more acurately (reduces speed violations).

**Increase maximum speed:** Due to some other fixes the maximum speed could be increased from 48 MPH to 49.5 MPH.

**Reduced maximum speed:** While doing a lane change, the simulator sometimes outputs a `max velocity` or `max acceleration` violation. Due to this the maximum speed was reduced from 49.5 MPH to 49 MPH. This is still an inprovement to the fist submission, where even 48 MPH produced many `max velocity` or `max acceleration` violations.

**Performance improvements:** Make some step sizes greater or simplify some calculations with a approximation, to increase performance and have less violations in the simulation.

**Better waypoints for spline:** During lane change there were some problems with the spline. Tried many different sets of waypoints. Now it seems quite good most of the time.

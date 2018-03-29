# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Goals

The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Project Outcome

[Video](https://youtu.be/pE9BCLVr0ik)

[Snapshot0](img/Screenshot0_pathplanning.png)

[Snapshot1](img/Screenshot1_pathplanning.png)


## Available Data

* Main car's localization 

* Sensor fusion data: a list of all other car's attributes on the same side of the road. (No Noise)

* Map list of waypoints around the highway. 

* Previous path data given to the Planner


### Main car's localization Data (No Noise)

["**x**"] The car's x position in map coordinates

["**y**"] The car's y position in map coordinates

["**s**"] The car's s position in frenet coordinates

["**d**"] The car's d position in frenet coordinates

["**yaw**"] The car's yaw angle in the map

["**speed**"] The car's speed in MPH


### Sensor Fusion Data

["sensor_fusion"] is a 2D vector for each car, [ id, x, y, vx, vy, s, d]

**id**: A car's unique ID

**x**: car's x position in map coordinates

**y**: car's y position in map coordinates

**vx**: car's x velocity in m/s

**vy**: car's y velocity in m/s

**s**: car's s position in frenet coordinates

**d**: car's d position in frenet coordinates

The vx, vy values can be useful for predicting where the cars will be in the future. For instance, if we assume that the tracked car kept moving along the road, then its future predicted Frenet s value will be its current s value plus its (transformed) total velocity (m/s) multiplied by the time elapsed into the future (s).

### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


### Previous path data given to the Planner

Note: Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time. 

["**previous_path_x**"] The previous list of x points previously given to the simulator

["**previous_path_y**"] The previous list of y points previously given to the simulator


#### Previous path's end s and d values 

["**end_path_s**"] The previous list's last point's frenet s value

["**end_path_d**"] The previous list's last point's frenet d value


## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

3. We use sensor fusion data to avoid collision. We first detect a car in the same lane, ahead of the main car, and then predict where the car will be in the future.  We can calculate the distance of the car from it, which is mentioned previously. Depending on how close it is, we deaccelerate the main car by a factor of 5m/s. We have three steps of deacceleration: "close", "too close", and "emergency". In "close" where the distance between the main car and the car in the same lane ahead of it is less than 30 meters, we reduce the relative velocity by 5m/s. In "too close" where the distance is less than 20 meters, we reduce it by 2 x 5m/s. In "emergency" where the distance is less than 10 meters, we reduce it by 3 x 5m/s. Whenver the lane is clear, we accelerate by 5m/s.

4. When the main car gets close to a car ahead in the same lane, it changes the lane to the left or to the right. Beforehand, it checks out whether the target lane has enough space to get in. We use indicators of "left_available" and "right_available" set to true as default. If there are cars within the range in the space, the corresponding indicator is set to false. So, a lane change is made as long as "right_available" or "left_available" is true. If both are true, then "left_available" has priority.

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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
## References

[Spline](http://kluge.in-chemnitz.de/opensource/spline/) is a helpful resource for creating smooth trajectories via interpolation

[Term3 Simulator](https://github.com/udacity/self-driving-car-sim/releases) contains the Path Planning Project.

[Introduction to Robotics #4: Path-Planning](http://correll.cs.colorado.edu/?p=965)

[Path Planning](http://www.coppeliarobotics.com/helpFiles/en/pathPlanningModule.htm)

[The path planning problem in depth](https://www.cs.cmu.edu/afs/cs/project/jair/pub/volume9/mazer98a-html/node2.html)

[RoboRealm](http://www.roborealm.com/help/Path_Planning.php)

[A discussion on What is the difference between path planning and motion planning?](https://robotics.stackexchange.com/questions/8302/what-is-the-difference-between-path-planning-and-motion-planning)

[Excellent Tutorial on A* Robot Path Planning](http://robotshop.com/letsmakerobots/excellent-tutorial-a-robot-path-planning)

[Path Planning in Environments of Different Complexity](https://www.mathworks.com/help/robotics/examples/path-planning-in-environments-of-different-complexity.html)

[Introduction to robot motion: Robot Motion Planning](http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/18-robot-motion-planning.pdf)

[Introduction to robot motion: Path Planning and Collision Avoidance](http://ais.informatik.uni-freiburg.de/teaching/ss10/robotics/slides/16-pathplanning.pdf)

[How to Debug Using GDB](http://cs.baylor.edu/~donahoo/tools/gdb/tutorial.html)

[GNU GDB Debugger Command Cheat Sheet](http://www.yolinux.com/TUTORIALS/GDB-Commands.html)
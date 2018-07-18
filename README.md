# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

---
## My Results

### Video
Here's a video of my results for this project.

Click image below for the YouTube video.

[![1 lap without incidents](https://img.youtube.com/vi/87kG1EezW1M/hqdefault.jpg)](https://youtu.be/87kG1EezW1M)

The video consists of autonomous driving of 1+ lap around the test track, with the following highlight points:
- [0:20](https://www.youtube.com/watch?v=87kG1EezW1M&t=20s) - encounters 1st set of traffic
- [1:20](https://www.youtube.com/watch?v=87kG1EezW1M&t=80s) - encounters 2nd set of traffic _(formation road block)_
- [2:10](https://www.youtube.com/watch?v=87kG1EezW1M&t=130s) - [2:50](https://www.youtube.com/watch?v=87kG1EezW1M&t=170s) - encounters several single vehicle traffic
- [4:00](https://www.youtube.com/watch?v=87kG1EezW1M&t=240s) - encounters 3rd set of traffic _(formation road block)_
- [5:00](https://www.youtube.com/watch?v=87kG1EezW1M&t=300s) - encounters 4th set of traffic _(Traffic changed lane to block, after ego vehicle has already maneuvered. This forced ego vehicle to make corrective maneuver.)_
- [5:30](https://www.youtube.com/watch?v=87kG1EezW1M&t=330s) - encounters last set of densely packed traffic _(formation road block with dynamic traffic speed changes, forcing ego vehicle to constantly re-evaluate the optimal path and make corrective maneuvers.)_

Note that:
- The green line is the commanded trajectory, extending 1 second ahead.
- The path planner is predicting and evaluating 3 seconds ahead in the future (beyond the green dots of the commanded trajectory).
- Given the option, the path planner has a preference to pass on the left instead of right.
- The path planner currently only scans 1 lane on either sides of the current lane. Thus if there's a better path 2 lanes away, it will not find it until the vehicle has moved to a lane next to such path.

The background soundtrack is from the arcade game of my childhood, [Spy Hunter](https://en.wikipedia.org/wiki/Spy_Hunter)!

### General Approach

I started the project by following the [Udacity project walkthrough and Q&A video](https://www.youtube.com/watch?v=7sI3VHFPP0w), which provided a good starting point with a 'working' trajectory generator that follows a chosen lane on the highway but without much traffic path planning capability.

I then:
* organized code base to make things more objected oriented,
* cleaned up some algorithms that were kind of messy/fragile/kludgy
* started adding & trying out new ideas, features and capabilities

All the while testing continuously to make sure existing functionalities are retained and not affected, and verified new added capabilities performed as intended.

I also intend on re-using this project code base on other future vehicle autonomy project ideas, thus designed it with that purpose in mind... (instead of 'moving fast' just to pass this course).

### Software Class Layout
It was noticed the `main.cpp` starter code contains numerous global functions. This is not an ideal layout for a extensive project like this, thus they were re-organized into `UitlFunctions.h` and `Track` class. Efforts were also taken to verify proper working of these functions from the starter code. That effort was not wasted...

#### 1. UtilFunctions.h
This include file contains many useful inlined global helper functions and constants. Many of these used to originally be in main.cpp starter code, and were moved here for clarity and organization.

#### 2. Track class
The `track` class holds knowledge about the closed-circuit track that the car will be driving on. It contains waypoints (WPTs) that define the centerline of the track, with 3 lanes on each sides of the centerline.

The WPTs data contains (x, y) global coordinates for each WPT, as well as their Frenet-S distance. Additionally, a (dx, dy) unit vector in global coordinate is also provided to define the direction of the Frenet-D at each WPT.

Methods to convert between global (x,y) and Frenet (s,d) are provided, as well as methods to find both the closest WPT and next WPT (based on orientation). These used to reside in `main.cpp` as global functions and are now class methods with direct access to the class WPT data, thus reducing the need to pass-in the WPT data as function parameters.

##### Cleaned up vector math
The original provided `getFrent()` function was cleaned up using vector geometry math. Vector cross-product was used to directly calculate both the magnitude and sign (+ or -) of the Frenet-d value, and vector dot-product was used for Frenet-s value. Further more, excessive for-looping was removed for computational efficiency.

#### 3. TrafficManager class
The `TrafficManager` class holds knowledge about all the known (sensed) traffic vehicles on the track, as well as predictions of each vehicle's future pose.

For each frame, `'sensor_fusion'` data from the simulator is passed to the `TrafficManager` class, where an instance of `Vehicle` class (described below) is created to represent each traffic vehicle's current pose (using `Pose.h`), and then each `Vehicle` instance is asked to create a prediction of its future pose. The `TrafficManager` then holds onto all these predictions.

#### 4. Pose.h
This include file contains a `Pose` struct which holds all the state data of a vehicle an an instance in time (ie: pose). It also contains a `FSM_state` enum that defines the Finite State Machine (FSM) states of the vehicle's path planner.

*The Behavior Planning classroom lesson code (which is the starting point of my project algorithm) used a `string` datatype to represent the vehicle's FSM `state`. My design changed the datatype from `string` to `enum` to help with clarity. But more importantly, it allows typo errors to be caught at compile-time, and it avoided the use of '[magic numbers](https://en.wikipedia.org/wiki/Magic_number_(programming)' to signify the FSM state.*

This file is meant to be included by any classes that will handle data pertaining to a vehicle's pose or FSM states so that they have the definition of these complex data types. Thus this file sort of acts as part of the interface (API).

#### 5. Vehicle class
The `Vehicle` class represents a vehicle that is on the road. It holds the pose of itself via a `Pose` data structure, and it knows how to propagate that pose into the future.

#### 6. EgoVehicle class
The `EgoVehicle` class derives from the `Vehicle` base class, thus contains all the base class capabilities. The `EgoVehicle` additionally contains a `PathPlanner` object.

#### 7. PathPlanner class
The `PathPlanner` class implements the core functionality of this project. It is associated with an `EgoVehicle` instance to get its `pose` data. It contains the Finite State Machine (FSM) where each state represents a different vehicle behavior. It generates **'predictive trajectory'** for each of these behavior states. These *'predictive trajectory'* contains only 2 `poses`: 1 present & 1 future. From each of these *'predictive trajectories'*, a **cost** is assigned based on their progress on the overall mission. The behavior with the lowest cost is then chosen as the next FSM state.

Finally, a **'control trajectory'** that defines the vehicle's future path is then generated, to be send to the simulator to control the ego vehicle's motion. This *'control trajectory'* must be smooth enough to avoid excessive acceleration and jerk.

The **'look ahead time'** for the *'predictive trajectory'* and the *'control trajectory'* could be different, and are different. For prediction, it looks 3.0 [sec] into the future; while for control, a trajectory 1.0 [sec] into the future is created.

**Note** that everything within behavior planning of *'predictive trajectories'* are calculated in Frenet (s,d) coordinates. While *'control trajectory'* generates output in global (x,y) coordinates.

Since the overall mission of the project is to navigate forward through traffic as fast as possible, while staying under the 50 [mph] speed limit; and each *'predictive trajectory'* are already formulated to not violate constrains (such as avoiding collision, staying with the 3 permissible lanes, adhering to acceleration and jerk limits, etc)... The **cost function** is then simply based on the forward progress only. The behavior having the lowest cost (ie: making the most forward progress) is chosen, and a 'control trajectory' is then finally build with specifications for that behavior.

The 'control trajectory' specifications are simply:
- new commanded speed,
- new commanded lane, and
- \# of previous frame path points to re-use, *set to 10pts (@50Hz interval) for this project*

The 'control trajectory' generation will ensure smooth vehicle transition of these commands.

### Algorithm Functional Allocations

#### Traffic Predictions

#### Behavior Planning

#### Trajectory Generation

##### Lateral Path Control


##### Speed Control

I tried to implement a 2nd-order lag filter, like in the block diagram below. And choosing the gains (`gain_a`, `gain_j`) judicially using engineering judgment.

>[replace below ASCII diagram with a nice block diagram?]
```
/*
 *          V_err         A_cmd   A_err          Jerk      Accel       Vel
 * Vin --->o----->[gain_a]----->o------>[gain_j]---->[1/S]--+--->[1/S]--+--> Vout
 *     (+) ^                (+) ^                           |           |
 *         |(-)                 |(-)   Accel_feedback       |           |
 *         |                    +---------------------------+           |
 *         |                                                            |
 *         |    Velocity_feedback                                       |
 *         +------------------------------------------------------------+
 */
```

With the associated 2nd order (Laplace transform) transfer function as:

<a href="https://www.codecogs.com/eqnedit.php?latex=\frac{V_{out}}{V_{in}}=\frac{G_{a}G_{j}}{S^{2}&plus;G_{j}S&plus;G_{a}G_{j}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\frac{V_{out}}{V_{in}}=\frac{G_{a}G_{j}}{S^{2}&plus;G_{j}S&plus;G_{a}G_{j}}" title="\frac{V_{out}}{V_{in}}=\frac{G_{a}G_{j}}{S^{2}+G_{j}S+G_{a}G_{j}}"/></a>

where:

* Natural frequency (Omega_n)   = `sqrt( gain_a * gain_j )`
* Damping ratio (zeta)          = `0.5 * gain_j / Omega_n`

Thus with:

* `gain_a` = 0.403 [1/sec]
* `gain_j` = 0.9 [1/sec]

The filter dynamics are:
* Omega_n = 0.602 [rad/sec]
* Zeta    = 0.747 [Non-Dimensional] <----**under-damped, oscillatory!!!**

However, I was having trouble debugging out the long-period speed oscillation and ending up spending too much time on controller design, instead of project path planning design.

Thus I **abandoned this approach**, for now. And instead implemented a much simpler **"limited acceleration speed controller"**.

>[Insert a block diagram here]

By limiting the max acceleration to 5.0 [m/s^2], and implementing it as a 'max-speed-change-per-frame' limit (`m_dv_accel_lim`) of 0.1 [m/s] (= 5.0*0.02). **This limits how much the speed could change per frame.** This yields a max jerk of only 2.5 [m/s^3] for a large amplitude step speed input. ONLY when a large step speed input is immediately followed by a large step in the opposite direction, does the max jerk reach 10.0 [m/s^3].

---
## Below are instructions provided by Udacity

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab v1.2](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x, y, s, dx, dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

* ["x"] The car's x position in map coordinates, [m]
* ["y"] The car's y position in map coordinates, [m]
* ["s"] The car's s position in frenet coordinates, [m]
* ["d"] The car's d position in frenet coordinates, [m]
* ["yaw"] The car's yaw angle in the map, [deg], (+)=yaw left from X to Y axis
* ["speed"] The car's speed in [MPH]

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

* ["previous_path_x"] The previous list of x points previously given to the simulator, [m]
* ["previous_path_y"] The previous list of y points previously given to the simulator, [m]

#### Previous path's end s and d values

* ["end_path_s"] The previous list's last point's frenet s value, [m]
* ["end_path_d"] The previous list's last point's frenet d value, [m]

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's:
* car's unique ID
* car's x position in map coordinates, [m]
* car's y position in map coordinates, [m]
* car's x velocity in [m/s]
* car's y velocity in [m/s]
* car's s position in frenet coordinates, [m]
* car's d position in frenet coordinates, [m]

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 10 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

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

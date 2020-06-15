# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Overview
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. The vehicle must always stay below a **50 mph speed limit** and must also maintain **acceleration <10 m/s^2** and **jerk <10 m/s^3** while **staying in the lanes** to prevent violation incidents that reset the distance and time counters.The vehicle drives by moving to each (x,y) coordinate in the path every 20 ms without any filtering.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


**Video of the project find**
[here](https://youtu.be/Zi09YAU431U)


**20 Miles of high way driving without any incedent**
![20 Miles of high Driving](/images/20_miles_high_way_driving.png)



## Prject Overview

![project overview](/images/control_overview.png)

1. **Sensor Fusion** - Gather and process data about the vehicle's state and the surrounding vehicles
2. **Prediction** - Estimate where the surrounding vehicles will be over a finite time horizon
3. **Behavior Planning** - Decide the vehicle's driving intent and target operating conditions
4. **Trajectory Generation** - Make a path trajectory that achieves the behavior target safely
5. **Control** - Actuate the vehicle to follow the planned path trajectory

#### 1. Sensor Fusion

In this project, the simulator provides some sensor data already processed for localization of the vehicle's global (x,y) position, the surrounding vehicles' global (x,y) positions and (vx,vy) velocities, and the previous path coordinates that have not been driven yet.  A map of sparse waypoints around the freeway track is also provided.  However, some additional processing is needed to prepare the data for path planning.

For this section, the following steps were implemented:

1. Process previous ego car path to determine where in the path the ego car is now
2. Use received (x,y) to update the ego car's state (convert to Frenet with custom functions)
3. Process detected cars within sensor range (to realistically limit available info)
4. Group detected car ID's by lane # for easier lookups

In order to simplify the coordinate system, all (x,y) coordinates are converted to Frenet (s,d) coordinates**, where **s** is the tangential distance along the road centerline and **d** is the normal (lateral) distance from the road centerline.  The original provided Frenet conversion functions were not accurate enough to generate smooth paths from direct (s,d)->(x,y) conversion, so a **custom Frenet conversion implementation** was made instead to avoid the need for spline interpolation of the path trajectory.  Splines were only used to interpolate the sparse map waypoints to a higher resolution.

The Frenet (s,d) coordinates were then used to estimate each vehicle's state.



#### 2. Prediction (line 126 to line 184)

Once the detected surrounding vehicles' states are known, their estimated paths can be predicted over a finite time horizon (~1.5 sec) with some probabilities that account for the prediction uncertainty.  These **predictions are needed to be able to choose a trajectory that considers the possible risk of collision with these surrounding vehicles at some future point in the path**.

For this section, the following step was implemented:

1. Predict detected car trajectories over fixed time horizon for each possible behavior with associated probabilities

Some estimations were made about the probabilities of a surrounding vehicle keeping it's lane or changing lanes and at what speed it would maintain based on the other vehicles around it.  The surrounding vehicle's start/end state acceleration for the predicted time horizon was assumed to be zero.

#### 3. Behavior Planning(line 187 to line 213)

The behavior planning section performs the highest level decision making to choose a target behavior including target lane, target intent, target speed, and target time to achieve them.  The surrounding vehicles are considered when setting these targets, but possible collisions are not factored in so that the ideal overall target is not restricted.

For this section, the following steps were implemented:

1. Decide **target intent** based on the target lane using a **Finite State Machine** with the following states:
*Keep Lane, Plan Lane Change Left, Plan Lane Change Right, Lane Change Left, Lane Change Right*
3. Decide **target time** for the planned path
4. Decide **target speed** for the end of the planned path


The target speed is limited to follow the car ahead and also considers the speed of a car in the target lane when changing lanes.



#### 4. Trajectory Generation(line 222 to line 341)
A smooth trajestory is calculated using a spline that contains some previous path points of the ego vehicle and some future points from the map. The actual future path points of the ego vehicle are derived from the spline. This helps to avoid jerk.

In order to avoid abrupt changes of the velocity, we incrementally increase or decrease the distance between the points of the path.

##  Project Rubric 

### The car is able to drive at least 4.32 miles without incident..
The car drive 20 miles with out any incedent 
Please find the below image 
![image](/images/20_miles_high_way_driving.png)

### The car drives according to the speed limit.
The car drive  velocty in between 30 to 49.5 MPH 

### Max Acceleration and Jerk are not Exceeded.

Maximum acceleration and jerk not exceed(observed)

### Car does not have collisions.

No collisions observed

### The car stays in its lane, except for the time between changing lanes.

Car always stay inside the lanes except car change the lanes to overcode the traffic with in 3 seconds 

### The car is able to change lanes

Car is able to change the lanes as per preiction and 
observed changing lanes all possible lanes. 

Please check video [here](https://youtu.be/Zi09YAU431U)



### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```


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


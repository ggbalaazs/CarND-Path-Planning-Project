# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Path Planning

The goal of this project is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane virtual highway with other traffic. The planner should be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion and map data provided by the simulator. Smooth trajetories mean that the car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

### Code Structure

The files `path_planning.h` and `path_planning.cpp` contains planner and related car classes. Utility functions are restructured to `utils.h` and `utils.cpp`. The spline tool `spline.h` is included from [here](http://kluge.in-chemnitz.de/opensource/spline/).

### Reflection

##### Prediction
Constant velocity model is used for predicting traffic car positions along the road (`path_planner.cpp`, line 122). To avoid car collisions a buffer zone is defined, forward 30m - backward 6m paths must be free to allow a lane change. But for safety reasons actual car positions must also match this criteria (`path_planner.cpp`, lines 165-174).

##### Behavior Planning
The above mentionned buffer zones might allow a possible lane change. But there is only need for that if our car is getting blocked by traffic.  If the car in front of us is closer than 30m, lane changing options are considered. If it is not possible our new target speed will match the front vehicle. If the distance is less than 10m, braking is forced no matter the target speed setting. When both sides are safe for a lane change, the one with more free distance forward will be selected. The behavior planning can be found in `path_planner.cpp`, lines 136-221.

##### Path Generation
Path generation is based on cubic splines with natural boundary conditions (2nd derivatives are zero) and linear extrapolation. Its anchor points are last known car position and three future positions. The last known car position is either the current position or the previously calculated path end position (`path_planner.cpp`, lines 31-52). The future positions are derived from Frenet coordinates,  30/60/90 meters ahead along s, and lane-specific d offset (`path_planner.cpp`, lines 56-65). The distance 30m is between future points. I was experimenting with longer values when planning lane changes for smoother paths e.g. 40m. The anchor points are transformed from global x-y coords to car-local x-y coords during fitting the spline and sampling its points based on car velocity (`path_planner.cpp`, lines 67-107). Keeping previously calculated path points - which are not processed by simulator - will ensure smoother driving behavior. Path calculation is limited to 50 points.

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

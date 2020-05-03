[main]: https://github.com/dringakn/CarND-Path-Planning-Project/blob/master/src/main.cpp
[//]: # "Image References"
[image1]: ./examples/Result.png "Final Result"
[video1]: ./examples/Path_Planning_Project.gif "Video"

# Path Planning Project

The goal of this project is to safely navigate around a highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data are provided, there is also a sparse map list of waypoints around the highway. The car tries to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The other cars will try to change lanes too. The car avoid colliding other cars as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car is able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it takes a little over 5 minutes to complete one loop. Also the car does not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

![Result][video1]

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

The ![main][main] contains the C++ code of the project.

### Dependencies

- cmake >= 3.5
  - All OSes: [click here for installation instructions](https://cmake.org/install/)
- make >= 4.1
  - Linux: make is installed by default on most Linux distros
  - Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  - Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
- gcc/g++ >= 5.4
  - Linux: gcc / g++ is installed by default on most Linux distros
  - Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  - Windows: recommend using [MinGW](http://www.mingw.org/)
  - [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  - Run either `install-mac.sh` or `install-ubuntu.sh`.
  - If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```

### Map

The map of the highway is in data/highway_map.txt. Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Simulator.

You can download the [Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/t3_v1.2) which contains the Path Planning Project.

To run the simulator on Linux, first make the binary file executable with the following command:

```shell
chmod +x term3_sim.x86_64
```

Afterwards, you can execute the simulator with the following command:

```shell
./term3_sim.x86_64
```

#### Variables Description

Here are the important information provided by the simulator:

`x`: The car's x position in map coordinates

`y`: The car's y position in map coordinates

`s`: The car's s position in frenet coordinates

`d`: The car's d position in frenet coordinates

`yaw`: The car's yaw angle in the map

`speed`: The car's speed in MPH

`previous_path_x`, `previous_path_y`: The processed points are removed from the previous list. Previous list of x, y points previously given to the simulator

`end_path_s`, `end_path_d`: The previous path's last point's in frenet coordinates, s and d values

`sensor_fusion`: Sensor Fusion Data, a list of obstacles (other cars) attributes on the same side of the road without noise. A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds (50 Hz). The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

A really helpful resource for doing this project and creating smooth trajectories was using [Spline](http://kluge.in-chemnitz.de/opensource/spline/), the spline function is in a single hearder file is really easy to use.

---

## Reflection

The implementation of the path planning is within the h.onMessage lambda function. Maximum speed limit, Maximum acceleration limit, and gap from the front car can be read from the command line and passed as reference to the lambda function.

Inside the ![main.cpp][main] the line-numbers `93-99` determine the lane number of the car on the road. Line-number `107-138` are used to detect obstacles (other cars) on the same side of the road. It determines the lane-numbers of the obstacles and set one of the three obstacle flags (front, left, right) based upon car s and d values (freenet coordinates).
Line-numbers `140-155` calcualates the car speed based upon the obstacle information. Line-numbers `157-183` are used for trajectory generation. Four reference waypoints are created for spline interpolation at lines `186-197` while the actual filtered spline points are genearted between line `217` to `235`.

As an improvement, a PID controller may be used to control vehicle velocity. Another improvement which can be made to the obstacle detection logic is to incorporate a finite state machine. Furthermore, the C++ code can be further improved using object oreiented approach.

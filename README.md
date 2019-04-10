# CarND-Path-Planning-Project
A project for the Self-Driving Car Engineer Nanodegree Program, ending May 21st, 2019.

## Running the project
``` bash
# First, download and run the simulator (instructions in further sections)

# Then, build the path planner:
mkdir build
cd build
cmake ..
make

# From project root, run the build path planner:
build/path_planning
```

## Comments
This project took me more than 3 weeks to complete, mostly because of the incorrect functions, data given to us. I have attempted several ways of planning the path, but eventually ended up slightly hacking around the solution. A full writeup with images and error graphs will be available on my website soon, at [https://linasko.github.io/portfolio/](https://linasko.github.io/portfolio/).

The next section documents the model for generating the paths. Below that, you will find the default readme, that explains more about the project.

## Model
### Initial attempts and discovery of error in the simulator
Before talking about the model, it might be useful to understand the context of the solution. The path selection problem is not that difficult to solve if you can estimate the problems you will encounter. This was not the case here, unfortunately.

While developing a quintic polynomial-based method of finding the optimal path wasn't hard at all, as wasn't dynamic lane changing, I never expected issues that I encountered afterwards. The car, while simply going straight, after around a minute or so of running time, would suddenly phase away from the track at high speeds, in a seemingly random direction. It took me a few weeks, but I managed to narrow it down to the `getFrenet` function. Apparently, even on a fairly straight road, with robot nearly following a straight line according to the simulator `x,y` coordinates, the `s,d` coordinates produced by `getFrenet` show a path leaning to the side, [with a disjunction (sheet 2)](https://docs.google.com/spreadsheets/d/1lPbdZdSkJSWSoUNa4XnZJV4dy5lJHc67ybA1e7k5vDs/edit?usp=sharing), that was causing the speed to suddenly become negative as the car was calmly driving along a straight road at high speed. Yay. Had to recode the whole solution.

The problem then was that the polynomial-based solution relied heavily on the s coordinates. Still, using splines wasn't too problematic, except for figuring out how to nicely set the speed. The problem was that there was another fault with the simulator. The speed of our car was sometimes wrong. For example, in meters per second, the speed at every tick could go like this:
```
16.0 -> 16.5 -> 17.0 -> 11.5 -> 18.0 -> 18.5
```
Yay again. I will have to find a way to use `x` and `y` to compute the speed myself. Still, I lost a few days looking for this issue, but it wasn't as complicated as the first one. After I cached the car speeds after every path point, it was pretty easy to get the right path afterwards.

### The Path Generation Model
Here's how my path generation code works:

It all stars in `main.cpp`, which initializes the map waypoint info and connects the asynchronous responses form the simulator to the handler function in `pathPlanner.cpp`. From here on, everything will happen in `pathPlanner.cpp`.

Generally, as can be in `planPath` (line 50), whenever we receive a response from the simulator, we:
* Add some of its response data to history. While still adapted to a legacy solution, we store some positions from the previous path in a double ended queue.
* Check if a lane change is needed and change the target lane if so.
* Plan a path to a distant point ahead of the car. And return the path to the simulator.

The path is then generated as follows. First, we wish to fit a spline between our start and end point. The main concern here is that it might differ significantly from out previous trajectory. So to make it smooth and inline with the past car poses, it we basically add the following points ot it:
* All the points in our car history (up to 100!), that are far enough apart from each other (otherwise I found that the splines 'burst' outwards, to the left and right of the car.). Note that these historical points are saved at the end of each run, before the path is sent.
* The starting `x,y` point of the car
* A couple of points form the previously planned path, as returned by the simulator
* Two distant points at the end of the path, selected according to the maximum possible distance we can travel in 2.5 seconds. Here S coordinate is used, that was established to be intermittently erroneous. Still, given that this target point is far, the error matters less, assuming the point's location is approximately correct.

The points are then transformed to the car's frame of reference. The main purpose of this is to make the x coordinates sorted, as that's what the spline library requires. The points are then used ot create a spline.

Now, we generate the path of more or less the distance to the target point. The maximum speed that we allow ourselves is 45.00 MPH, but we also restrict ourselves to the speed of the car in front. We iterate over every time point (line 208) on the path, and referring to the previous speed, compute the next one by adding or subtracting constant speedup value (manually tuned). We cache the speeds generated after every timepoint, as that's where we get our current speed from. As mentioned in the previous section, we definitely can't trust the speed, returned from the simulator, and this is our workaround. After all this, the car points are converted to world coordinates and returned.

### Lane changing
The other component maybe worth mentioning is the lane change bit. Briefly, we check if we've not carried out a lance change recently, look to the nearby lanes, and change if the attainable speed there is higher than in our current lane, and there's no car blocking our way.

### Other bits
The implementation is held together by smaller bits of functionality, but at this point, it would be a bit difficult to describe every workaround and functionality used. Especially if I also went into detail the the functions for the alternative solutions - 3 ways of computing kinematics, several ways of planning the path, potentially better car predictions (I'm not using any here). Feel free to browse through the code yourself - I hope you will find it sufficiently documented and understandable.

## Everything Else
From here on, you can read the initial readme as it was given to us, with no changes made by me,

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

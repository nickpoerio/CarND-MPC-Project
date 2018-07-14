# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program


## Model Description
### State, actuators, equations
A model predictive control has been implemented using the following 6 states: position coordinates (x,y), speed, heading, cross track error and heading error.
The actuations are the steering angle and the gas/brake pedal, which for this application, for simplicity, is straightly related to the acceleration/deceleration.
The update equations follow the single track model kinematics, taking particular care of the steering wheel sign inversion, as suggested in the project instructions.

### Goal
I set myself a little bit harder goal: not only follow the path, but trying to complete the lap as fast as possible. For this purpose, the ideal condition would be to have
some additional info about the vehicle, like mass, yaw inertia and tyre characteristics, in order to use a single track dynamic model. As it is not the case, I tryed to apply a simpler
approach, which still makes the job, that I will explain with the cost function description.

### Cost Function
I used the following costs to build the cost function: cross track, heading and speed squared errors, squared actuations, squared actuations variations. In fact, besides minimizing the path and speed error,
it is important not to choose too sharp actuations, which could be unefficient, unconfortable or even cause of instability.
The core idea to go fast is to properly define a reference speed. With the informations I have, the simplest way to do that is to set a maximum lateral acceleration and calculate
the curvature of the path, from which calculate the speed. For the curvature calculation I need:  
- the forseen x distance at each step  
- the first and the second derivative of the polynomial.
  
The detailed equations can be found in the code (lines 56-62 of the MPC.cpp file).    
The constant max lateral acceleration assumption is simplistic because:  
- maximum lateral acceleration is deeply affected by the longitudinal one, due to tire properties  
- aero effects caused by the rear wing could increase much the maximum lateral acceleration achievable at high speed.
  
This being said, I accepted these drawbacks and tried to get the best from what I have! Further details will be provided in the next chapter.  


## Model Tuning
The tuning process has certainly not been straightforward, rather quite iterative. At the end of the process, the vehicle seems to drive slightly faster than in the video shown in the project
introduction, so that I can consider my peronal goal achieved. The detailed tuning steps follow.

### Horizon
I first decided to reduce the MPC horizon, 2s made the model a little bit harder to tune as they often included 2 curves (trajectory not easily described by a single polynomial).
Initially it was set to 1.5s, which was not that bad, and used this value to tune the cost function. Exploring things further, I then realized that 1s could be the best compromise between prediction
capabilities and computational cost and stability of the control (a two short horizon causes instability).

### Number of points
The number of prediction steps was chosen to guarantee an accurate description of control action and vehicle response, assuring a good computational cost. A frequency of 10 Hz enclosures very
well my dynamic constrains without weighing to much on the computation: a number of 10 points has been chosen with the 1s horizon.

### Cost Function Weights
My first tuning steps were performed at constant speed. I started with 30 mph and then went beyond up to 50 mph. This speed was enough to tune the path cost function and in particular the balance
between cross track error and heading error. I priviledged the heading error, so to be more stable at higher speed and properly "cut" the curves: this is done on purpose in order to better approximate
what a race driver would do and so to go faster. Hence, you will notice that the speed reference estimation is conservative from this point of view, but at the end it balances the neglection of the 
interaction with the longitudinal acceleration.
I kept the speed weight unchanged, while adjusted the actuation weights after the proper speed reference generation. I privileged the minimization of the actuation variation rather than of the actuation itself,
also because I am performing a race driving and extreme actuations, especially in pedals, are ok.

### Speed Reference
I tuned the maximum acceleration value so to reach the best compromise between the speed and the stability. There were solutions that achieved a maximum speed of 101 mph, but not quite stable. My current
solution is fast and stable, achieving almost 97 mph max speed in the second lap. Some slight refinement is possible, but it wouldn't add much to the overall result. The tuning value of 150 actually includes
the square of the conversion factor m/s to mph: the reference max lateral acceleration is about 3g, which could be somehow justifiable by the wind aero effect.  
You will also notice a non trivial bias to the curvature (8e-3), used not only to avoid division by zero, but also to reduce the amount of the speed error in quasi-straight line condition (you would have enormous reference speed!), 
which would cause instability in the path control.  

## Model Preprocessing

### Waypoints
I took the waypoints and projected to the car coordinate frame (see lines 105-115 in main.cpp). Then I used a 3rd order polynomial to interpolate the points. This is the minimum order to guarantee a curvature variation, which also determines
a reference speed variation. If instead of a polynomial, a spline were needed (longer predictions), a 5th order would have been necessary, in order to assure continuity in the curvature variation which is the reference for vehicle yaw acceleration.

### State, Latency and Actuators
The initial state (0 displacements and measured initial errors) has been projected 0.1 s ahead using single track model kinematics equations (lines 126-146 in main.cpp), so that the latency of the system minimally affects the controller performance.
Actuator values are calculated by the optimizer and passed to the vehicle block, with the proper sign.


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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

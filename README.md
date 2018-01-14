# **Udacity Self-Driving Car Enginer Nanodegree - *PID Control Project***

## Introduction

This project is to "implement a PID controller in C++" to maneuver the vehicle of the lake race track simulator around the track. The simulator provides cross-track error (CTE), speed, and steering angle via local uWebSocket. The implementation is to control steering of the wheels and throttle via PID controller to drive safely but also "to channel our inner Vin Diesel".

## Implementation

The PID controller code consists of the following files:

* json.hpp   (not changed)
* PID.h      (PID header file with PID errors and coefficients, parameter for simulation control, twiddle and debug parameter)
* PID.cpp    (Routines for PID error calculation and for twiddling the PID coefficients)
* main.cpp   (Steering and throttle calculation, uWebSocket communication, reset of simulation if necessary)

The PID controller is implemented by the function PID::UpdateError() where the errors of the PID elements are updated based upon CTE and by the function PID::TotalError() which calculates the final error for PID control.

To enable twiddling, the function PID::UpdateError() has been extended and additional routines PID:Reset() and PID::ResetRun() have been implemented.

The twiddling algorithm of the lecture has been adopted. It has a twiddle-mode (*twiddle_allowed*) where twiddling is enabled in general and a twiddle phase (*twiddle*) during which the twiddling of the coefficients is performed. In twiddle phase, it is checked whether a violation (*cte* > *maxcte*) occurs in step range [1, *maxstep*]. If there has been a violation, the simulation will be resetted (*PID::Reset()*) and the PID coefficients are continously changed according to the twiddle algorithm. If it passes *maxstep* without violation, twiddling of the coefficients will be disabled until a violation beyond *maxstep* happens. In this case, *maxstep* is set to the actual step size and the twiddling process begins again (*PID::ResetRun()*) if the lap couldn't be finalized (*step* <= *roundsteplimit*). If a violation occured and the lap could be finalized, the twiddling phase will be enabled and simulation will be resetted (*PID::Reset()*). Once a full lap has been more or less finalized (*step* > *roundsteplimit*), the user will be informed and it is upon him to decide whether he has a good parameter set or if he wants continue the testing.

## Reflection: Effect of the P, I, D components

### P- or Proportional-Component

This component has the most directly observable effect on the car's behavior. It causes the car to steer proportional (and opposite) to the car's distance from the lane center (CTE). Due to its nature, it causes the car to swing around the center line. The swinging increases over time as can be seen in the video: ./Videos/P_control.m4v.

### I- or Integral-component

To reduce the bias offset from lane center, the I-component is used. For a driving vehicle, it reduces the drift of its wheels. In this simulator, the vehicle seems to be modelled ideally, so no I-component is needed.

### D- or Differential-component

The swinging of the P-component can be reduced by the D-component. It causes the vehicle to approach the lane center smoothly without swinging. An example is given in the video: ./Videos/PID_control.m4v. The vehicle is controlled by PID-components but the I-component is set to 0.0. So, actually it is a PD-controlled vehicle.

To fulfill the project requirements, the hyperparameter were tuned according the described twiddle algorithm above. First, the twiddling algorithm has been used to optimize the hyperparameter of the steering at constant throttle (0.3). Maxcte was set to 2.5 to get a very smooth driving line nearly at the center of the track. Second, with the found hyperparameter set for steering, the twiddle algorithm was used to find the right hyperparameters for throttle. Finally, maxcte was set to 4.0 and the found steering parameters were twiddled with fixed throttle parameter to find hyperparameters for maximum speed. The found hyperparameters were used with different throttle calculations (TotErrP1 - TotErrP3) to maximize even more for speed.

The following PID-coefficients were found:

* Steering: P,I,D = 0.153144, 0.0, 6.95
* Throttle: P,I,D = 1.0, 0.0, 0.0

For this simulator, a PD-controller for steering and a P-controller for throttle seem to be sufficient enough. With this coefficients, the vehicle could reach up to 70 MPH with a safe course. Even 80 MPH were reached but often the car crashed after an uncertain number of laps (between 3 and >30, non-reproducable) due to timing-wise unreliable uWebSocket communication.

---

# **Original Udacity README**

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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


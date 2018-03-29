# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Visual Output
[PID-controlled GTA](https://youtu.be/94UMQNydGXg)

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Quick Overview of PID Controller

PID controller is a technique by to minimize the cross track error (CTE). The CTE represents the difference from a track or lane line. PID stands for Proportional, Integral, and Derivative. First, a control parameter P is adjusted proportional to the CTE. When the P is large, the vehicle quickly responds to the CTE, and vice versa. So, often than not, overshoots likely happen. To mitigate the overshooting, derivative of CTEs is counted in. By considering CTE change rates, the vehicle's responsiveness is controlled. The vehicle system may have a bias, caused by physical distortion or conversion. We need calibration for it. The controller achieves this by the integral of CTEs. 

## Tuning of Parameters

We start off by manually tuning P and D parameters first. Once the vehicle runs on track to a certain degree, we set I parameter to a reasonably large value e.g., 2. The parameters can be fine-tuned with twiddle() applied.

## Reference

1. [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
2. [Project rubric](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)

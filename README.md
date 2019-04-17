# **Project 8: PID Controller**

This document presents the work developed for the 8th project of the SDC Nanodegree.
____________________________________________

Table of Contents:
1. Project Introduction
2. Build Instructions
3. Reflection on how to tune the controller parameters
4. Results

## 1. Project Introduction
In this project you'll revisit the lake race track from the Behavioral Cloning Project. This time, however, you'll implement a PID controller in C++ to maneuver the vehicle around the track!

The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

The speed limit has been increased from 30 mph to 100 mph. Get ready to channel your inner Vin Diesel and try to drive SAFELY as fast as possible! NOTE: you don't have to meet a minimum speed to pass.

### 1.1. Input Data
Here is the relevant data provided from the Simulator to the C++ Program

["cte"] The car's cross track error with respect to the desired position in the track.

["angle"] The car's steering angle in degrees

["speed"] The car's speed in MPH

### 1.2. Output Data

The algorithm produces 2 variables to drive the car:

["steering_angle"] The angle to steer the car, in order to minimize the cte. This value is provided by the PID controller.

["throttle"] Amount of positive or negative acceleration to change the car's speed.

## 2. Build instructions
### 2.1. Dependencies

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

### 2.2. Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## 3. Reflection on how to tune the controller

The tuning of the PID controller parameters was done manually. For this, the following steps were executed:

* First, I reduced the car acceleration, which initially is fixed to 0.3. This causes that the car is always accelerating and makes it difficult to control its steering. To reduce its speed, I defined a low target velocty of 10 MPH, and adjusted the throttle (starting now at 0) so the car's speed was around this value:

```
// Change in the car's trhottle to follow the target speed
if (speed > target_vel){
  throttle -= 0.1;
} else{
  throttle += 0.05;
} 
```

* Then, now that the car's speed is somehow low and stable, I started to manually set the PID parameters. I set the *kd* and *ki* gains to 0, and the *kp* gain to 1, and started modifying this value. This proportional gain controls how the car responds to not being in the center of the lane, and I saw that the car is very sensible to this parameter. Small values of *kp* produced significant steering changes that result in the car making abrupt changes in its direction. Taking this into account, I started lowering the proportional gain, until the car started driving a little more smoothly.
* After *kp* is set, the next value to consider was the derivative gain *kd*. This is a very important parameter in this case, since it helps to reduce the cte while also reducing overshoots, which for the car means less driving from one side to the other and more close to the center of the lane. This gain has the biggest weight in the controller.
* The last gain to tune was *ki*. This parameter helps to reduce the accumulative cte along the lane, which helps the car achieve the center of the lane despite bias in its actuators. However, this is a track with many curves, and that increases a lot the accumulative cte. Due to this, small values of *ki* can generate great impacts on how the car makes turns. Taking this into account, the value for *ki* in the model was set very small.
* Once the controller parameters were tuned, the car was driving a lot better, but still at a speed of 10 MPH. The objective now was to increase te car's velocity, mantaining it driving inside the race track. For this, the `target_velocity` parameter was slowly increased. A important thing to take into account is that the car should try to minimize its throttle, to save energy (gas, braking) and have a more comfortable trajectory. In order to do so, a hysteresis controller was introduced, to mantain the car's speed within a desired margin around the target velocity. If the car is within this margin, it does not accelerate, and will speed up/down if it is under/over the margin:

```
// Change in the car's trhottle to follow the target speed
if (speed > target_vel + target_vel_margin){
  throttle -= 0.1;
} else if(speed < target_vel + target_vel_margin) {
  throttle += 0.05;
} else {
  // Don't accelerate if the car is within the target speed tolerance zone
  throttle = 0.0;
}
```

Also, if the car is making a step turn, it will slow down for safety: 

```
// Reduce the car's speed if it is making a steep turn
if (fabs(angle) > 8.0){
  throttle -= 0.2;
  // Allow the car to break a little
  throttle = clip_signal(throttle,-0.08,0.5);
}
```

Both outputs of the program (steering angle and throttle) are cliped before passing them to the simulator, so they don't exceed some upper and lower limits.

* Finally, introducing these changes in the car's speed made it necessary to do some final tuning on the PID parameters. The final parameters chosen were:

```
// PIDs parameters
kp = 0.15;
ki = 0.002;
kd = 3.1;
```

## 4. Results
This project's main criteria to determine if the implementation of the controller is successful is that:

*No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).*

Below, I show a video of the car driving around the race track. It still has some small undesired "zigzag" behavior at some points, but in overall it drives safely around the track. Using a dynamic model representation of the car and some tuning algorithm like Twiddle, it could be possible to get better values for the PID parameters.

<img src="media/1.gif" width="600">

In general, the car is able to achieve speeds over 40 MPH in a relatively safe manner. However, the fastest the car goes, the more visible is its "zigzag" effect.














    

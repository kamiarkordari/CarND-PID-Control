 In this project I implemented a PID controller in C++ to maneuver an autonomous vehicle around a track.

### Overview
In this project we build a PID controller and tune the PID hyperparameters. Then we test the solution in the simulator. The goal is to drive the vehicle succesfully around the track. The speed limit is 100 mph.

The simulator sends cross-track error, speed and angle to the PID controller using WebSocket and it receives the steering angle that is a normalized value between -1 and 1 and the throttle to drive the car.

### Background

### The Code
##### Simulator
The simulator provides the cross track error (CTE) and the velocity (mph) and the controller uses those information to compute the appropriate steering angle.


##### Running the Code
This project involves a Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).


### Twiddle Algorithm
I applied the twiddle algorithm to update the parameters automatically. The main concept is to methodically vary each of the parameters and measure the resulting difference in error to determine if increasing or decreasing the value was improving the overall error.
- I set the param deltas to initialize to 10% of the seed value.
-  I used a sample size of 100. That means 100 measurements are made between each twiddle of a hyperparameter.
- Twiddle incorporates a tolerance value as the hyperparameters are tuned. I used 0.2.

Pseudo code for implementing the Twiddle algorithm is as follows:

```Python
function(tol=0.2) {
    p = [0, 0, 0]
    dp = [1, 1, 1]
    best_error = move_robot()
    loop untill sum(dp) > tol
        loop until the length of p using i
            p[i] += dp[i]
            error = move_robot()

            if err < best_err
                best_err = err
                dp[i] *= 1.1
            else
                p[i] -= 2 * dp[i]
                error = move_robot()

                if err < best_err
                    best_err = err
                    dp[i] *= 1.1
                else
                    p[i] += dp[i]
                    dp[i] *= 0.9
    return p
}
```
### Dependencies

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


### Reflection
**What is the effect each of the P, I, D components?**

- The proportional portion of the controller steers the car toward the center line. If used alone, the car quickly overshoots the central line and goes out of the road. Higher values of the coefficient tend to increase the weaving of the car. Lower values of the coefficient (e.g. 0.0) tend to increase the time to adjust and hardly change the steering direction.

- The differential portion counteract the overshoot caused by the proportional portion to by generating a smooth approach to the center line. Higher values of the coefficient (e.g. 15) tend to decrease the smoothness of the steering.

- The integral portion helps to eliminates a non-zero permanent bias/error in the output. Higher values of the coefficient (e.g 0.5) tend to overshoot the reference.
**How were the final hyperparameters chosen?**

The parameters were chosen manually to make sure the car can drive straight.  add the proportional and the car start going on following the road but it starts overshooting go out of it. Then add the differential to try to overcome the overshooting. The integral part only moved the car out of the road; so, it stayed as zero. After the car drove the track without going out of it, the parameters increased to minimize the average cross-track error on a single track lap. The final parameters where [P: 1.5, I: 0.0, D: 2.5].

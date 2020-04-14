[result]: ./media/pid.gif "Short Gif"
![result]

In this project I implemented a PID controller in C++ to maneuver an autonomous vehicle around a track and used Twiddle algorithm to automatically adjust PID parameters on the fly.

### PID Controller
##### Goal
The goal is to build a PID controller and tune its parameters for an autonomous vehicle as it is driven around a track in the simulator.

##### Simulator
The simulator sends cross-track error (CTE), speed, and angle to the PID controller using WebSocket. The controller computes the appropriate steering angle (that is a normalized value between -1 and 1) and the throttle to drive the car back to the simulator.

The Simulator can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).


##### The Effect of the P, I, D Components
- **The proportional** portion of the controller results in a steering that is proportional to the distance the car is off the center of the lane. If used alone, the car is pushed to the center and quickly overshoots the middle of the lane and may go out of the road. Higher values of the coefficient tend to increase the oscillation of the car. Lower values of the coefficient result in increased time to correct the error.

- **The differential** portion counteract the overshoot caused by the proportional portion by generating a smooth approach to the center of the lane. Higher values of the coefficient (higher than 10) tend to result in oscillation of he car around the center line.

- **The integral** portion helps to eliminates a non-zero permanent error in the output. Higher values of the coefficient (higher than 0.01) tend to overshoot the reference.


##### Parameter Selection
I chose the parameters manually to make sure the car can drive without going off the track and then I used twiddle algorithm to fine tune those parameters.

Here is my process to manually find the initial PID parameters:
- I first added the proportional component while keeping other parameters zero. I chose `Kp = 0.1` that resulted in the car following the road while not overshooting out of it.
- Then I added the differential parameter to try to overcome the overshooting. `Kd = 1` worked fine.
- The integral part was not improving the error in any significant way. I chose `Ki = 0.0001` for this parameter.


Here are some sample videos to show the effect of the Kp and Kd components on the driving quality.

  Kp  |  Ki  | Kd |  Video Link
------|----- | ---|-------------
 1 | 0  |  0 | [Video Link](https://www.youtube.com/watch?v=NtbGrdZbvcM)
 0.1 | 0  |  0 | [Video Link](https://www.youtube.com/watch?v=WA-sdZhimBs)
 0.1 | 0  |  1 | [Video Link](https://www.youtube.com/watch?v=4uhZ9vBzcuE)
 0.1 | 0  |  10 | [Video Link](https://www.youtube.com/watch?v=utDTVqofqiA)



##### Twiddle Algorithm
I applied the twiddle algorithm to tune the PID parameters automatically. The main concept of twiddle is to methodically vary parameters one at a time and measure the resulting difference in error to determine if increasing or decreasing the value was improving the overall error.

The pseudo code for implementing the Twiddle algorithm is as follows:

```Python
function(tol=0.1) {
    p = [0.1, 0.0001, 1]
    dp = [0.1, 0.0001, 1]
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

- I set the parameter deltas to initialize to 100% of the initial values.
- With trail and error I found a set of parameters that create a reasonable response. I used those values as the initial PID parameters: `Kp = 0.1, Ki = 0.0001, Kd = 1`
- Between each twiddle parameter update the algorithm waits 1000 cycles to measure average error.
- Twiddle incorporates a tolerance value of 0.1 as the parameters are tuned.

After going one time around the track the parameters were adjusted to `Pk = 0.2, Pi = 0.0001, and Pd = 2`.

### Result
See a video of the final result by clicking on the image below.

[image-final-result]: ./media/screen_shot.PNG "Final Video Screenshot"
[![image-final-result]](https://www.youtube.com/watch?v=Rw_0oPXjEiQ)



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

# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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

## Discussions
### PID Controller
![PID Controller](img/pid.png)

#### P - Proportional
The P parameter steers the car according to the distance from the lane center. A large P will make the steering oscillate around the lane center.

#### I - Integral 
The I parameter is to compensate for biases. A large I will make steering turn too much and correcting the steering very slowly.

#### D - Differential
D parameter helps the cars to drive to the centerline smoothly.

### The Result
The following values are used for steering & throttle.

For steering, P value is adjusted for less oscillation and enough for turning.  D value is adjusted so that it can turn quickly enough. I is just big enough to compensate.
```
  //steering PID
  pid.Init(0.1, 0.001, 1.2);
```  

Similar to steering adjustment for throttle, however we like to have much smoother throttle changes than steering, a smaller P & D is used.
```
// throttle PID
  pid_throttle.Init(0.01, 0.0001, 0.1);
```
For throttle control, a slower target speed is also used for a larger angle turn, with min speed 10 and max speed 35.

```
    //Throttle value
    double target_speed = 35 - (3 * abs(angle));

    //set min speed
    if(target_speed < 0)
      target_speed = 10;
    else if (target_speed > 35)
      target_speed = 35;
```

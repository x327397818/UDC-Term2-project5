# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)

[Model]: ./output/Model.PNG "Model"
[Constraints]: ./output/Constraints.PNG "Constraints"


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


## Implementation

### The Model

The kinematic model state is ``[X, Y, psi, V, cte, epsi]``. X and Y are the coordinates of the vehicle. Psi is the host orientation, V is velocity. cte and epsi are the cross-track error and psi error.

Actuators are ``[delta, A]``, which are steering angle and acceleration.

Here is the model state update and constraints functions,

![alt text][Model]

![alt text][Constraints] 

### Timestep Length and Elapsed Duration (N & dt)

After serveral times of parameter tuning, I decided to use ``N = 10, dt = 0.1``. The other value tried are ``[5, 0.2] [15, 0.067] [20, 0.05]``.

### Polynomial Fitting and MPC Preprocessing

The waypoints coordinates are transformed from globle coordinates to vehicle coordinates. Here is the reference https://en.wikipedia.org/wiki/Rotation_of_axes

After transforming, the initial state of the vehicle will be ``[0, 0, 0, cte, epsi]`` since in the respect of vehicle, it start coordinates are [0, 0], and psi is 0 too.

### Model Predictive Control with Latency

Since there is Latency of 100 ms. I want to give the compensation on the initial state before put to MPC.solve.
Here is the equations I use to do compensation. Basically, I just add the state change in dt to the initial state.
```
                            cur_x = x0 + (v0 * cos(psi0) *dt)
							cur_y = y0 + (v0 * sin(psi0) *dt)
							cur_psi = psi0 + (v0 * steer_value / Lf * dt)
							cur_v = v0 + throttle_value * dt
							cur_cte = cte0 + v0 * sin(epsi0) * dt
							cur_epsi = epsi0 + v0 * steer_value / Lf * dt
```


## Simulation

Here is the output recording of the system.

[MPC](https://github.com/x327397818/UDC-Term2-project5/blob/master/output/self_driving_car_nanodegree_program%206_29_2018%2010_03_35%20PM.mp4)

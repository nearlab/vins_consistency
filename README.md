# VINS Consistency
Consistency of filters for VIsual Inertial Navigation Systems for Odometry and SLAM

## 3_code
* `slam.m`: Main file for SLAM
* `localize.m`: Main file for VIO only, known feature positions
* `dyn_x.m`: Dynamics of the system for SLAM or VIO
* `Phidot.m`: Dynamics of the State Transition matrix
* `meas_x.m`: Measurement equation for bearing, range or relative position measurement (SLAM and VIO)
* `delf_x.m`: Partial of dynamics
* `delh_x.m`: Partial of measurement equation
* `isInFOV.m`: Returns a boolean vector of features which are in the Camera FOV
* `rk4fixed.m`: Propagation with Runge-Kutta 4 with fixed time step
* `normAngle.m`: Bring the angle back to (-pi,pi]
* `plotting.m`: Plotting and simulation

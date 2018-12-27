# **Sensor Fusion using Extended Kalman Filter**
---

**In this project, I have worked on the development of Kalman Filter (KF) and Extended Kalman Filter (EKF) algorithm in C++ to fuse Lidar and Radar data for tracking object position and velocity (both in x and y directions).**

## Algorithm Architecture:
---
1. Initialize Kalman Filter and Extended Kalman Filter Matrices
2. Check if it is the first measurement ?
   * If yes, initialize state `x` and covariance matrices
   * If no, go to prediction step
3. Predeiction step
   * Compute elapsed time `dt`
   * Use `dt` to compute EKF new state transition matrix `F` and process noise covariance matrix `Q`
   * Predict the state vector `x` and state covariance matrix `P` (which indicates the uncertainty in the prediction step)
4. Based on the coming data is from Lidar or Radar sensor
   * If it is Lidar data, so set up Lidar matrices `R` and `H` and call the update function for Lidar, where `R` is the senor noise covariance matrix and `H` is the transformation matrix (function) from prediction subspace to measurement subspace.
   * If it is Radar data,
      * Linearize the the measurement function `H`
      * Set up Radar matrices `H` and `R`
   * Update state with new measurement
 5. Performace Evaluation
 

The following flow-chart summerizes the algorithm:
![ekf_flow_chart](https://i.imgur.com/nUtrxA7.png)

## Environment:
---
* Ubuntu 16.04 LTS
* Udacity Self-Driving Car Nano-Degree Term2 Simulator
* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4

For setting up the environmernment and how to use the simulator, check the [CarND-Extended-Kalman-Filter-Project](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)

If you need to know more about KF, please visit the lectures from [iLectureOnline](http://www.ilectureonline.com/lectures/subject/SPECIAL%20TOPICS/26/190) and the SDCND sensor fusion module.
## Conclusion
---
  * The EKF is tracking the object position and velocity and fusing data between two sensors
  * The Taylor serier and Jacobian matrix solved the problem of non-linearity
  * The perfomance ouput with RMSE values : [X= 0.08, Y, 0.09, VX= 0.44, VY = 0.48]
 

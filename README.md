Sensor Fusion using Extended Kalman Filter

In this project, I have worked on the development of Kalman Filter (KF) and Extended Kalman Filter (EKF) algorithm in C++ to fuse Lidar and Radar data for tracking object position and velocity (both in x and y directions).
Algorithm Architecture:

    Initialize Kalman Filter and Extended Kalman Filter Matrices
    Check if it is the first measurement ?
        If yes, initialize state x and covariance matrices
        If no, go to prediction step
    Predeiction step
        Compute elapsed time dt
        Use dt to compute EKF new state transition matrix F and process noise covariance matrix Q
        Predict the state vector x and state covariance matrix P (which indicates the uncertainty in the prediction step)
    Based on the coming data is from Lidar or Radar sensor
        If it is Lidar data, so set up Lidar matrices R and H and call the update function for Lidar, where R is the senor noise covariance matrix and H is the transformation matrix (function) from prediction subspace to measurement subspace.
        If it is Radar data,
            Linearize the the measurement function H
            Set up Radar matrices H and R
        Update state with new measurement
    Performace Evaluation
Conclusion

    The EKF is tracking the object position and velocity and fusing data between two sensors
    The Taylor serier and Jacobian matrix solved the problem of non-linearity
    The perfomance ouput with RMSE values : [X= 0.08, Y, 0.09, VX= 0.44, VY = 0.48]
    

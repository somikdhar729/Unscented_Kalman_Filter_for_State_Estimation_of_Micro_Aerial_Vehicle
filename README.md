# Unscented_Kalman_Filter_for_State_Estimation_of_Micro_Aerial_Vehicle

The objective of this project is to develop an Unscented Kalman Filter (UKF) that fuses the inertial data and the
vision-based pose and velocity estimation. The UKF is expected to capture the non-linearity of the system better,
but might require more runtime. The measurements are taken from two sources, the visual pose estimation in the
world frame, and the optical flow velocity in the camera frame, which needs to be carefully moved to the body frame
that is coincident with the IMU frame.

## Part 1: Fusion of Inertial and Visual Pose Estimation using UKF with Linear Measurement Model
We have used the Unscented Kalman Filter (UKF) to fuse the inertial data with the visual pose estimation. The
UKF was chosen as it can capture the non-linearity of the system better than the Extended Kalman Filter (EKF).
We used the IMU-driven model and the visual pose and velocity estimates as measurements.<br>
The visual pose estimation was expressed in the world frame, so the same linear measurement model used in project
1 is applied to this measurement.

### Methodology
The unscented Kalman filter (UKF) implementation utilizes the unscented transform to propagate sigma points through the nonlinear process model. This captures the drone's state dynamics more accurately than linearization techniques. <br>
  * Computes optimized sigma points from the state mean and covariance 
  * Propagates points through process model to predict next state 
  * Calculates predicted mean and covariance as weighted sums of propagated points

To improve state estimation, the process model has been augmented with additional error state variables: <br>

  * Gyroscope and accelerometer biases are modeled as random walks 
  * Gyroscope and accelerometer noise are modeled as independent random variables 
  
  
These additional error states are estimated alongside the core drone state variables like position and velocity

Uses a linear measurement model for simplicity:
  * Kalman gain fuses predicted state and measurement uncertainties
  * Applies gain to measurement residual to correct prediction
  * Updated state used for next prediction cycle
  

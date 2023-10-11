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


## Part 2: Fusing IMU and Optical Flow Data using UKF
Incorporating Optical Flow Velocity into IMU-derived State Estimation

In this phase of the project, our objective is to refine the state estimation process for the micro-aerial vehicle. We focus on utilizing the optical flow velocity as a crucial measurement, specifically aligning it with the IMU frame utilized in the previous project phase.

### Measurement Alignment and Model Integration
1. **Measurement Source**: We isolate the velocity component from optical flow, which is initially expressed in the camera frame.

2. **Coordinate Transformation**: Due to the disparity between the camera frame and the IMU frame used in the prior phase, we meticulously transform the optical flow velocity measurement to align with the body frame coincident with the IMU frame. This transformation is facilitated by a nonlinear measurement model, which considers the rotational relationship between the camera frame and the body frame (IMU frame).

  ** Nonlinear Measurement Model: This model efficiently maps the optical flow velocity from the camera frame to the body frame, accounting for the intricate transformation dynamics.

3. Integration with IMU-driven Model: Having successfully aligned the measurement to the body frame, we seamlessly integrate it with the IMU-driven model employed in the initial phase of the project.

### State Estimation with Unscented Kalman Filter (UKF)
* Estimation Methodology: Leveraging the power of the Unscented Kalman Filter (UKF), we aim to estimate the system's state by amalgamating the refined optical flow velocity measurement with the IMU-driven model.

* Advantages of UKF: The UKF excels in capturing the inherent nonlinearity of the system, a superior alternative to a linear Kalman filter. Despite potential increased runtime, the UKF significantly enhances the accuracy and robustness of our state estimation.

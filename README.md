# InertialPoseLib

InertialPoseLib is a library for estimating **IMU State**

## Features
### IMU-based EKF State Estimation using Eigen Library
  - 24-DOF EKF for IMU state estimation and bias removal
    - Pos, Vel, Gyro, Acc, IMU Bias, Gravity ...
  - Supports external pose update
  - Supports Complementary Filtering for orientation estimation
  - Supports ZUPT through vehicle stop detection (TODO)

## How To Use

Refer to the example file in the `/example` folder: `example-PoseEstimation.cpp`

## Build Instructions

To build the InertialPoseLib library, follow these steps:

1. **Run CMake**:
   When building with CMake, you can use the `INERTIALPOSELIB_PRECISION` option to specify the precision of floating-point variables. For example:
   ```bash
   catkin_make -DINERTIALPOSELIB_PRECISION=1
   ```
   This option allows you to choose between different precision levels (1 for float, 2 for double, etc.).

## Links
- Example video: TODO
- Author: Jaeyoung Jo, wodud3743@gmail.com
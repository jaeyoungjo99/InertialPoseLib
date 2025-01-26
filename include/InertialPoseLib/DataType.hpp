/**
 * @file DataType.hpp
 * @author Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Data type file for InertialPoseLib
 * @version 0.1
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025
 */

#ifndef __DATA_TYPE_HPP__
#define __DATA_TYPE_HPP__
#pragma once

// STD header
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Libraries
#include <Eigen/Dense>

// Define Real type based on precision setting
#if REAL_PRECISION == 1
    using Real = float; // 1 = float
    using Matrix3 =  Eigen::Matrix3f;
    using Vector3 =  Eigen::Vector3f;
    using Quaternion =  Eigen::Quaternionf;
    using AngleAxis =  Eigen::AngleAxisf;
    using Affine3 =  Eigen::Affine3f;
    using MatrixX =  Eigen::MatrixXf;
#elif REAL_PRECISION == 2
    using Real = double; // 2 = double
    using Matrix3 =  Eigen::Matrix3d;
    using Vector3 =  Eigen::Vector3d;
    using Quaternion =  Eigen::Quaterniond;
    using AngleAxis =  Eigen::AngleAxisd;
    using Affine3 =  Eigen::Affine3d;
    using MatrixX =  Eigen::MatrixXd;
#elif REAL_PRECISION == 3
    using Real = long double; // 3 = extended
    using Matrix3 =  Eigen::Matrix3l;
    using Vector3 =  Eigen::Vector3l;
    using Quaternion =  Eigen::Quaternionl;
    using AngleAxis =  Eigen::AngleAxisl;
    using Affine3 =  Eigen::Affine3l;
    using MatrixX =  Eigen::MatrixXl;
#else
    using Real = double; // Default to double
    using Matrix3 =  Eigen::Matrix3d;
    using Vector3 =  Eigen::Vector3d;
    using Quaternion =  Eigen::Quaterniond;
    using AngleAxis =  Eigen::AngleAxisd;
    using Affine3 =  Eigen::Affine3d;
    using MatrixX =  Eigen::MatrixXd;
#endif


#define S_X 0
#define S_Y 1
#define S_Z 2
#define S_ROLL 3
#define S_PITCH 4
#define S_YAW 5
#define S_VX 6
#define S_VY 7
#define S_VZ 8
#define S_ROLL_RATE 9
#define S_PITCH_RATE 10
#define S_YAW_RATE 11
#define S_AX 12
#define S_AY 13
#define S_AZ 14
#define S_B_ROLL_RATE 15
#define S_B_PITCH_RATE 16
#define S_B_YAW_RATE 17
#define S_B_AX 18
#define S_B_AY 19
#define S_B_AZ 20
#define S_G_X 21
#define S_G_Y 22
#define S_G_Z 23

#define STATE_ORDER 24

#define GNSS_MEAS_ORDER 6 // x y z roll pitch yaw
#define INIT_STATE_COV 100.0

namespace InertialPoseLib {

    // EKF state
    typedef struct {
        Real timestamp{0};
        Vector3 pos{Vector3::Zero()};           // global
        Quaternion rot{Quaternion::Identity()}; // global
        Vector3 vel{Vector3::Zero()};           // global
        Vector3 gyro{Vector3::Zero()};          // local
        Vector3 acc{Vector3::Zero()};           // global
        Vector3 bg{Vector3::Zero()};            // bias gyro
        Vector3 ba{Vector3::Zero()};            // bias acc
        Vector3 grav{Vector3::Zero()};          // global gravity
        Quaternion imu_rot{Quaternion::Identity()};
    } EkfState;

    typedef struct {
        Real timestamp{0};
        Vector3 acc{Vector3::Zero()};
        Vector3 gyro{Vector3::Zero()};
    } ImuStruct;

    typedef struct {
        Real timestamp{0};
        Vector3 pos = Vector3::Zero();           // global
        Quaternion rot = Quaternion::Identity(); // global
        Matrix3 pos_cov = Matrix3::Identity();   // position covariance (3x3 matrix)
        Matrix3 rot_cov = Matrix3::Identity();   // rotation covariance (3x3 matrix)
    } GnssStruct;
    
} // namespace InertialPoseLib

#endif

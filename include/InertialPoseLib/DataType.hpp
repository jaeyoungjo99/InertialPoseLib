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
// #include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Libraries
#include <Eigen/Dense>

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
#define CAN_MEAS_ORDER 2  // v_local_x, yaw_rate
#define INIT_STATE_COV 100.0

namespace InertialPoseLib {

    // EKF state
    typedef struct {
        double timestamp{0};
        Eigen::Vector3d pos{Eigen::Vector3d::Zero()};           // global
        Eigen::Quaterniond rot{Eigen::Quaterniond::Identity()}; // global
        Eigen::Vector3d vel{Eigen::Vector3d::Zero()};           // global
        Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};          // local
        Eigen::Vector3d acc{Eigen::Vector3d::Zero()};           // global
        Eigen::Vector3d bg{Eigen::Vector3d::Zero()};            // bias gyro
        Eigen::Vector3d ba{Eigen::Vector3d::Zero()};            // bias acc
        Eigen::Vector3d grav{Eigen::Vector3d::Zero()};          // global gravity
        Eigen::Quaterniond imu_rot{Eigen::Quaterniond::Identity()};
    } EkfState;

    typedef struct {
        double timestamp{0};
        Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
        Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
    } ImuStruct;

    typedef struct {
        double timestamp{0};
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();           // global
        Eigen::Quaterniond rot = Eigen::Quaterniond::Identity(); // global
        Eigen::Matrix3d pos_cov = Eigen::Matrix3d::Identity();   // position covariance (3x3 matrix)
        Eigen::Matrix3d rot_cov = Eigen::Matrix3d::Identity();   // rotation covariance (3x3 matrix)
    } GnssStruct;
    
} // namespace InertialPoseLib

#endif

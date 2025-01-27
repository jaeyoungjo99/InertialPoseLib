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

#if !defined(INERTIALPOSELIB_PRECISION)
#  define INERTIALPOSELIB_PRECISION 2
#endif

namespace InertialPoseLib {

    class Math{
    public:
        // Define real type based on precision setting
        #if INERTIALPOSELIB_PRECISION == 1
            typedef float real;
        #elif INERTIALPOSELIB_PRECISION == 2
            typedef double real;
        #else
            typedef double real;
        #endif
    };

    typedef Math::real real;

    static constexpr int S_X = 0;
    static constexpr int S_Y = 1;
    static constexpr int S_Z = 2;
    static constexpr int S_ROLL = 3;
    static constexpr int S_PITCH = 4;
    static constexpr int S_YAW = 5;
    static constexpr int S_VX = 6;
    static constexpr int S_VY = 7;
    static constexpr int S_VZ = 8;
    static constexpr int S_ROLL_RATE = 9;
    static constexpr int S_PITCH_RATE = 10;
    static constexpr int S_YAW_RATE = 11;
    static constexpr int S_AX = 12;
    static constexpr int S_AY = 13;
    static constexpr int S_AZ = 14;
    static constexpr int S_B_ROLL_RATE = 15;
    static constexpr int S_B_PITCH_RATE = 16;
    static constexpr int S_B_YAW_RATE = 17;
    static constexpr int S_B_AX = 18;
    static constexpr int S_B_AY = 19;
    static constexpr int S_B_AZ = 20;
    static constexpr int S_G_X = 21;
    static constexpr int S_G_Y = 22;
    static constexpr int S_G_Z = 23;

    static constexpr int STATE_ORDER = 24;
    static constexpr int GNSS_MEAS_ORDER = 6; // x y z roll pitch yaw
    static constexpr real INIT_STATE_COV = 100.0;

    using Matrix3 = Eigen::Matrix<real, 3, 3>;
    using Vector3 = Eigen::Matrix<real, 3, 1>;
    using Quaternion = Eigen::Quaternion<real>;
    using AngleAxis = Eigen::AngleAxis<real>;
    using Affine3 = Eigen::Transform<real, 3, Eigen::Affine>;
    using MatrixX = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;

    // EKF state
    typedef struct {
        real timestamp{0};
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
        real timestamp{0};
        Vector3 acc{Vector3::Zero()};
        Vector3 gyro{Vector3::Zero()};
    } ImuStruct;

    typedef struct {
        real timestamp{0};
        Vector3 pos = Vector3::Zero();           // global
        Quaternion rot = Quaternion::Identity(); // global
        Matrix3 pos_cov = Matrix3::Identity();   // position covariance (3x3 matrix)
        Matrix3 rot_cov = Matrix3::Identity();   // rotation covariance (3x3 matrix)
    } GnssStruct;
    
} // namespace InertialPoseLib

#endif

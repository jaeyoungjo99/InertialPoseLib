/**
 * @file PoseEstimation.hpp
 * @author Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Pose Estimation functions for InertialPoseLib
 * @version 0.1
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025
 */

#ifndef __POSE_ESTIMATION_HPP__
#define __POSE_ESTIMATION_HPP__
#pragma once

#include "InertialPoseLib/DataType.hpp"
#include "InertialPoseLib/Utils.hpp"
#include "InertialPoseLib/LinearAlgebra.hpp"

namespace InertialPoseLib {

    struct PoseEstimationParams {
        double imu_gyro_std = 0.0001;
        double imu_acc_std = 0.001;
        double imu_bias_gyro_std = 0.0001;
        double imu_bias_acc_std = 0.0001;

        double imu_gravity = 9.81;

        double state_std_pos_m = 0.02;
        double state_std_rot_rad = 0.003;
        double state_std_vel_mps = 2.0;

        bool estimate_imu_bias = false;
        bool estimate_gravity = false;
    };

    class PoseEstimation {
    public:
        PoseEstimation(const PoseEstimationParams& params = PoseEstimationParams())
        {Reset(params);}

        ~PoseEstimation() {}

        void Reset(const PoseEstimationParams &params);

    public:
        void PredictImu(const ImuStruct& imu_input);
        void UpdateWithGnss(const GnssStruct& gnss_input, bool is_3dof = false);
        
        void PrintState() const; // TODO:

        EkfState GetCurrentState() const{return S_;}

    private:
        void ComplementaryKalmanFilter(const ImuStruct& imu_input);

        template <int MEAS_SIZE, int K_COLS>
        void UpdateEkfState(const Eigen::Matrix<double, STATE_ORDER, K_COLS>& K,    // Kalman Gain
                            const Eigen::Matrix<double, MEAS_SIZE, 1>& Y,           // Residual
                            Eigen::Matrix<double, STATE_ORDER, STATE_ORDER>& P,     // Covariance matrix
                            const Eigen::Matrix<double, MEAS_SIZE, STATE_ORDER>& H, // Observation matrix
                            EkfState& X);                                           // EKF State
        
    // variables
    private:
        EkfState S_;
        Eigen::Matrix<double, STATE_ORDER, STATE_ORDER> P_; // covariance

        std::mutex mutex_state_;
        double prev_timestamp_;
        bool reset_for_init_prediction_;

    // config
    private:
        bool estimate_gravity_;
        bool estimate_imu_bias_;

        double state_std_pos_m_;
        double state_std_rot_rad_;
        double state_std_vel_mps_;

        double imu_gyro_std_;
        double imu_acc_std_;
        double imu_bias_gyro_std_;
        double imu_bias_acc_std_;

        double imu_gravity_;
    };

} // namespace InertialPoseLib


#endif
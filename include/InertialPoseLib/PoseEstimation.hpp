#ifndef __POSE_ESTIMATION_HPP__
#define __POSE_ESTIMATION_HPP__
#pragma once

#include "InertialPoseLib/DataType.hpp"
#include "InertialPoseLib/Utils.hpp"
#include "InertialPoseLib/LinearAlgebra.hpp"

namespace InertialPoseLib {
    class PoseEstimation {
    public:
        PoseEstimation(double imu_bias_std_gyro = 0.0001, double imu_bias_std_acc = 0.0001, bool estimate_gravity = false)
        {Reset(imu_bias_std_gyro, imu_bias_std_acc, estimate_gravity);}

        ~PoseEstimation() {}

        void Reset(double imu_bias_std_gyro, double imu_bias_std_acc , bool estimate_gravity);

        void PredictImu(const ImuStruct& imu_input);
        void UpdateWithGnss(const GnssStruct& gnss_input, bool is_3dof = false);
        
        void PrintState() const;

        EkfState GetCurrentState() const{
            return S_;
        }

        void TestPrint() const{
            std::cout << "Test PoseEstimation" << std::endl;
        }

    private:
        void ComplementaryKalmanFilter(const ImuStruct& imu_input);

        template <int MEAS_SIZE, int K_COLS>
        void UpdateEkfState(const Eigen::Matrix<double, STATE_ORDER, K_COLS>& K,    // Kalman Gain
                            const Eigen::Matrix<double, MEAS_SIZE, 1>& Y,           // Residual
                            Eigen::Matrix<double, STATE_ORDER, STATE_ORDER>& P,     // Covariance matrix
                            const Eigen::Matrix<double, MEAS_SIZE, STATE_ORDER>& H, // Observation matrix
                            EkfState& X                                             // EKF State
        );


    private:
        EkfState S_;
        Eigen::Matrix<double, STATE_ORDER, STATE_ORDER> P_; // covariance

        double state_std_pos_m_ = 0.02;
        double state_std_rot_rad_ = 0.003;
        double state_std_vel_mps_ = 2.0;

        double imu_std_gyro_ = 0.00015;
        double imu_std_acc_ = 0.001;
        double imu_bias_std_gyro_;
        double imu_bias_std_acc_;

        double imu_gravity_ = 9.81;
        bool estimate_gravity_ = false;

        std::mutex mutex_state_;

        double prev_timestamp_;
        bool b_reset_for_init_prediction_;
    };

} // namespace InertialPoseLib


#endif
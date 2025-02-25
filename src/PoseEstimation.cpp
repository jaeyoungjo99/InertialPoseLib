/**
 * @file PoseEstimation.cpp
 * @author Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Pose Estimation functions for InertialPoseLib
 * @version 0.1
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025
 */

#include "InertialPoseLib/PoseEstimation.hpp"
#include <typeinfo> // Include typeinfo for type identification

namespace InertialPoseLib {
    void PoseEstimation::Reset(const PoseEstimationParams &params,
        Vector3 init_pos, Quaternion init_rot){

        imu_gyro_std_ = params.imu_gyro_std;
        imu_acc_std_ = params.imu_acc_std;

        imu_bias_gyro_std_ = params.imu_bias_gyro_std;
        imu_bias_acc_std_ = params.imu_bias_acc_std;

        imu_gravity_ = params.imu_gravity;

        state_std_pos_m_ = params.state_std_pos_m;
        state_std_rot_rad_ = params.state_std_rot_rad;
        state_std_vel_mps_ = params.state_std_vel_mps;

        estimate_imu_bias_ = params.estimate_imu_bias;
        estimate_gravity_ = params.estimate_gravity;

        // State initialization using configuration
        S_.pos = init_pos;
        S_.rot = init_rot;
        S_.vel.setZero();
        S_.gyro.setZero();
        S_.acc.setZero();
        S_.bg.setZero();
        S_.ba.setZero();
        S_.grav = Vector3(0.0, 0.0, imu_gravity_);
        
        // Covariance initialization using configuration
        P_ = MatrixX::Identity(STATE_ORDER, STATE_ORDER) * INIT_STATE_COV;

        P_(S_ROLL_RATE, S_ROLL_RATE) = imu_gyro_std_ * imu_gyro_std_;
        P_(S_PITCH_RATE, S_PITCH_RATE) = imu_gyro_std_ * imu_gyro_std_;
        P_(S_YAW_RATE, S_YAW_RATE) = imu_gyro_std_ * imu_gyro_std_;
        P_(S_AX, S_AX) = 1.0;
        P_(S_AY, S_AY) = 1.0;
        P_(S_AZ, S_AZ) = 1.0;

        P_(S_B_ROLL_RATE, S_B_ROLL_RATE) = imu_bias_gyro_std_ * imu_bias_gyro_std_;
        P_(S_B_PITCH_RATE, S_B_PITCH_RATE) = imu_bias_gyro_std_ * imu_bias_gyro_std_;
        P_(S_B_YAW_RATE, S_B_YAW_RATE) = imu_bias_gyro_std_ * imu_bias_gyro_std_;
        P_(S_B_AX, S_B_AX) = imu_bias_acc_std_ * imu_bias_acc_std_;
        P_(S_B_AY, S_B_AY) = imu_bias_acc_std_ * imu_bias_acc_std_;
        P_(S_B_AZ, S_B_AZ) = imu_bias_acc_std_ * imu_bias_acc_std_;

        P_(S_G_X, S_G_X) = imu_bias_acc_std_ * imu_bias_acc_std_;
        P_(S_G_Y, S_G_Y) = imu_bias_acc_std_ * imu_bias_acc_std_;
        P_(S_G_Z, S_G_Z) = imu_bias_acc_std_ * imu_bias_acc_std_;

        prev_timestamp_ = 0.0;
        reset_for_init_prediction_ = true;

    }

    void PoseEstimation::PredictImu(const ImuStruct& imu_input){
        std::unique_lock<std::mutex> lock(mutex_state_, std::try_to_lock);

        if (lock.owns_lock() == false) {
            // If the lock is not owned, do not perform prediction (other process is accessing state)
            // Measurement Update priority
            return;
        }
        
        // Validate imu_input data
        if (imu_input.timestamp <= prev_timestamp_) {
            // Invalid timestamp: it should be greater than the previous timestamp
            return;
        }

        // Check for reasonable gyro and accel values (example thresholds)
        const real gyro_threshold = 1000.0; // Example threshold for gyro
        const real accel_threshold = 50.0;   // Example threshold for acceleration

        if (imu_input.gyro.norm() > gyro_threshold || imu_input.acc.norm() > accel_threshold) {
            // Invalid gyro or acceleration values
            return;
        }

        real cur_timestamp = imu_input.timestamp;
        S_.timestamp = cur_timestamp;
        

        if (reset_for_init_prediction_ == true) {
            prev_timestamp_ = cur_timestamp;

            reset_for_init_prediction_ = false;

            return;
        }

        // Calculate prediction dt
        real dt = cur_timestamp - prev_timestamp_;

        if (fabs(dt) < 1e-6) {
            return; // Not new data
        }

        EkfState ekf_state_prev = S_;

        // State rotation quat to matrix
        Matrix3 G_R_I = S_.rot.toRotationMatrix();

        // Gyroscope reading corrected for bias
        Vector3 corrected_gyro = imu_input.gyro - ekf_state_prev.bg;
        Quaternion delta_rot = ExpGyroToQuat(corrected_gyro, dt);
        S_.rot = (ekf_state_prev.rot * delta_rot).normalized();

        // Compensate IMU Bias and Gravity
        Vector3 corrected_accel = imu_input.acc - ekf_state_prev.ba;
        Vector3 accel_global = G_R_I * corrected_accel - ekf_state_prev.grav;

        // Predict Pose, Velocity
        S_.pos += ekf_state_prev.vel * dt + 0.5 * accel_global * dt * dt;
        S_.vel += accel_global * dt;

        // Use prior state of gyro and acc
        S_.gyro = corrected_gyro;
        S_.acc = accel_global;

        // Use prior state of bias and gravity
        S_.bg = ekf_state_prev.bg;
        S_.ba = ekf_state_prev.ba;
        S_.grav = ekf_state_prev.grav;

        // Process model matrix (Q) generation
        Eigen::Matrix<real, STATE_ORDER, STATE_ORDER> Q = Eigen::Matrix<real, STATE_ORDER, STATE_ORDER>::Zero();

        // Allocate std in Q matrix
        // Position, Rotation, Velocity, Accel
        Q.block<3, 3>(S_X, S_X) = Matrix3::Identity() * std::pow(state_std_pos_m_, 2) * dt * dt;
        Q.block<3, 3>(S_ROLL, S_ROLL) =
                Matrix3::Identity() * std::pow(state_std_rot_rad_, 2) * dt * dt;
        Q.block<3, 3>(S_VX, S_VX) = Matrix3::Identity() * std::pow(state_std_vel_mps_, 2) * dt * dt;
        Q.block<3, 3>(S_ROLL_RATE, S_ROLL_RATE) =
                Matrix3::Identity() * std::pow(imu_gyro_std_, 2) * dt * dt;
        // Q.block<3, 3>(S_AX, S_AX) = Matrix3::Identity() * std::pow(imu_acc_std_, 2) * dt * dt;

        // IMU Bias, Gravity
        Q.block<3, 3>(S_B_ROLL_RATE, S_B_ROLL_RATE) =
                Matrix3::Identity() * std::pow(imu_bias_gyro_std_, 2) * dt * dt;
        Q.block<3, 3>(S_B_AX, S_B_AX) =
                Matrix3::Identity() * std::pow(imu_bias_acc_std_, 2) * dt * dt;
        Q.block<3, 3>(S_G_X, S_G_X) = Matrix3::Identity() * std::pow(imu_bias_acc_std_ * 10.0, 2) * dt * dt;


        // auto diagonal = P_.diagonal().transpose();
        // std::cout <<"\nPOS: "<< diagonal[0] << " " << diagonal[1] << " " << diagonal[2] << std::endl;
        // std::cout <<"ROT: "<< diagonal[3] << " " << diagonal[4] << " " << diagonal[5] << std::endl;
        // std::cout <<"VEL: "<< diagonal[6] << " " << diagonal[7] << " " << diagonal[8] << std::endl;
        // std::cout <<"GYR: "<< diagonal[9] << " " << diagonal[10] << " " << diagonal[11] << std::endl;
        // std::cout <<"ACC: "<< diagonal[12] << " " << diagonal[13] << " " << diagonal[14] << std::endl;
        // std::cout <<"BGY: "<< diagonal[15] << " " << diagonal[16] << " " << diagonal[17] << std::endl;
        // std::cout <<"BGA: "<< diagonal[18] << " " << diagonal[19] << " " << diagonal[20] << std::endl;
        // std::cout <<"GRA: "<< diagonal[21] << " " << diagonal[22] << " " << diagonal[23] << std::endl;

        // Covariance (P) propagation using CV model
        Eigen::Matrix<real, STATE_ORDER, STATE_ORDER> F = Eigen::Matrix<real, STATE_ORDER, STATE_ORDER>::Identity();

        // Position partial differentiation
        F.block<3, 3>(S_X, S_VX) = Matrix3::Identity() * dt; // ∂pos / ∂vel

        if(estimate_imu_bias_){
            F.block<3, 3>(S_X, S_B_AX) = -0.5 * G_R_I * dt * dt; // ∂pos / ∂ba (including global transform)
            
            // Rotation partial differentiation
            F.block<3, 3>(S_ROLL, S_B_ROLL_RATE) = -PartialDerivativeRotWrtGyro(corrected_gyro, dt); // d rot / d bg
            F.block<3, 3>(S_ROLL_RATE, S_B_ROLL_RATE) = -Matrix3::Identity(); // ∂gyro / ∂bg

            // Velocity partial differentiation
            F.block<3, 3>(S_VX, S_B_AX) = -G_R_I * dt; // ∂vel / ∂ba (including global transform)
            // F.block<3, 3>(S_AX, S_B_AX) = -G_R_I; // ∂acc / ∂ba (including global transform)
        }

        if (estimate_gravity_) {
            // Only Z axis
            F(S_Z, S_G_Z) = -0.5 * dt * dt; // ∂z / ∂gz
            F(S_VZ, S_G_Z) = -dt;           // ∂vz / ∂gz
            // F(S_AZ, S_G_Z) = -1.0;          // ∂az / ∂gz
        }

        // Covariance matrix
        P_ = F * P_ * F.transpose() + Q;

        prev_timestamp_ = cur_timestamp;
        

        ComplementaryKalmanFilter(imu_input);

        return;
    }

    void PoseEstimation::UpdateWithGnss(const GnssStruct& gnss_input, bool is_3dof) {
        std::lock_guard<std::mutex> lock(mutex_state_);

        // Main Algorithm

        // Observation matrix: H
        Eigen::Matrix<real, GNSS_MEAS_ORDER, STATE_ORDER> H;
        H.setZero();
        H.block<3, 3>(0, 0) = Matrix3::Identity(); // Position
        H.block<3, 3>(3, 3) = Matrix3::Identity(); // Rotation

        // Measurement model: Z_state (state position and rotation)
        Eigen::Matrix<real, GNSS_MEAS_ORDER, 1> Z_state;
        Z_state.head<3>() = S_.pos; // Position

        // Measurement vector: Z (from gnss_input)
        Eigen::Matrix<real, GNSS_MEAS_ORDER, 1> Z;
        Z.head<3>() = gnss_input.pos;

        // Measurement covariance matrix: R
        Eigen::Matrix<real, GNSS_MEAS_ORDER, GNSS_MEAS_ORDER> R =
                Eigen::Matrix<real, GNSS_MEAS_ORDER, GNSS_MEAS_ORDER>::Zero();
        R.block<3, 3>(0, 0) = gnss_input.pos_cov;
        R.block<3, 3>(3, 3) = gnss_input.rot_cov;

        // Observation covariance matrix: S (GNSS_MEAS_ORDER, GNSS_MEAS_ORDER)
        Eigen::Matrix<real, GNSS_MEAS_ORDER, GNSS_MEAS_ORDER> S = H * P_ * H.transpose() + R;

        // Kalman Gain: K (STATE_ORDER * GNSS_MEAS_ORDER)
        Eigen::Matrix<real, STATE_ORDER, GNSS_MEAS_ORDER> K = P_ * H.transpose() * S.inverse();

        // Residual: Y (for position and rotation in quaternion)
        Vector3 res_angle_euler = CalEulerResidualFromQuat(S_.rot, gnss_input.rot);

        Eigen::Matrix<real, GNSS_MEAS_ORDER, 1> Y;
        Y.head<3>() = Z.head<3>() - Z_state.head<3>(); // Position residual
        Y.tail<3>() = res_angle_euler;                 // Rotation residual as angle-axis vector

        if (is_3dof == true) {
            Eigen::Matrix<real, 3, STATE_ORDER> H3 = H.block<3, STATE_ORDER>(0, 0);
            Eigen::Matrix<real, 3, 3> S3 = H3 * P_ * H3.transpose() + R.block<3, 3>(0, 0);
            Eigen::Matrix<real, STATE_ORDER, 3> K3 = P_ * H3.transpose() * S3.inverse();
            Eigen::Matrix<real, 3, 1> Y3 = Y.head<3>();

            UpdateEkfState(K3, Y3, P_, H3, S_);
        }
        else {
            UpdateEkfState(K, Y, P_, H, S_);
        }

        return;
    }

    void PoseEstimation::PrintState() const{
        
        std::cout.precision(6);
        std::cout << "State:\t" << std::endl;
        std::cout << "Position:\t" << S_.pos.transpose() << std::endl;
        std::cout << "Rotation:\t" << RotToVec(S_.rot.toRotationMatrix()).transpose() * 180.0/M_PI <<" deg"<< std::endl;
        std::cout << "Velocity:\t" << S_.vel.transpose() << std::endl;
        std::cout << "Gyro:\t" << S_.gyro.transpose() * 180.0/M_PI <<" deg/s" << std::endl;
        std::cout << "Acc:\t" << S_.acc.transpose() << std::endl;
        std::cout << "Bias Gyro:\t" << S_.bg.transpose() * 180.0/M_PI <<" deg/s" << std::endl;
        std::cout << "Bias Acc:\t" << S_.ba.transpose() << std::endl;
        std::cout << "Gravity:\t" << S_.grav.transpose() << std::endl;
    }
    /*
        Complementary Filter:
        Purpose: Correct rotation error by estimating gravity direction
        1. Remove bias estimate from sensor input
        2. Compensate centrifugal force
        3. Estimate gravity direction
        4. Correct rotation error
    */
    void PoseEstimation::ComplementaryKalmanFilter(const ImuStruct& imu_input) {
        // Detect dynamic state through acceleration magnitude check
        const Eigen::Matrix<real, 3, 1> vec_meas_acc = imu_input.acc - S_.ba; // Bias-corrected acceleration

        // Compensate centrifugal force
        Matrix3 G_R_I = S_.rot.toRotationMatrix();
        Eigen::Matrix<real, 3, 1> vec_grav_removed_acc = G_R_I * vec_meas_acc - S_.grav;
        Eigen::Matrix<real, 3, 1> vec_grav_removed_meas_acc = vec_meas_acc - G_R_I.transpose() * S_.grav;

        real acc_mag = vec_grav_removed_meas_acc.norm() + 1.0;

        // 1. Calculate measurement value (z) - use compensated acceleration for centrifugal force
        Eigen::Matrix<real, 3, 1> gravity_direction = vec_meas_acc.normalized();
        Eigen::Matrix<real, 2, 1> z; // roll, pitch measurement value
        z << std::atan2(gravity_direction.y(), gravity_direction.z()), // roll
            -std::asin(gravity_direction.x());                         // pitch

        // 2. Extract roll, pitch of current state (h(x))
        Eigen::Matrix<real, 2, 1> h_x = RotToVec(S_.rot.toRotationMatrix()).head<2>();

        // 3. Calculate Innovation (residual Y)
        Eigen::Matrix<real, 2, 1> innovation = z - h_x;

        // Normalize roll, pitch to the range [-π, π]
        innovation(0) = NormAngleRad(innovation(0));
        innovation(1) = NormAngleRad(innovation(1));

        // 4. Calculate Measurement Jacobian (H)
        Eigen::Matrix<real, 2, STATE_ORDER> H = Eigen::Matrix<real, 2, STATE_ORDER>::Zero();
        H(0, S_ROLL) = 1.0 ;  // ∂roll_meas/∂roll
        H(1, S_PITCH) = 1.0; // ∂pitch_meas/∂pitch

        // 5. Set Measurement noise covariance (R)
        Eigen::Matrix<real, 2, 2> R = Eigen::Matrix<real, 2, 2>::Zero(); 

        // Final measurement noise covariance TODO: Adjust based on the magnitude of the compensated acceleration
        R(0, 0) = M_PI * 1000.0 * acc_mag;   // roll
        R(1, 1) = M_PI * 1000.0 * acc_mag; // pitch


        // 6. Kalman gain
        Eigen::Matrix<real, 2, 2> S = H * P_ * H.transpose() + R;                             // Innovation covariance
        Eigen::Matrix<real, STATE_ORDER, 2> K = P_ * H.transpose() * S.inverse(); // Kalman gain

        // 7. EKF State Update
        UpdateEkfState<2, 2>(K, innovation, P_, H, S_);
    }

    template <int MEAS_SIZE, int K_COLS>
    void PoseEstimation::UpdateEkfState(const Eigen::Matrix<real, STATE_ORDER, K_COLS>& K,
                        const Eigen::Matrix<real, MEAS_SIZE, 1>& Y,
                        Eigen::Matrix<real, STATE_ORDER, STATE_ORDER>& P,
                        const Eigen::Matrix<real, MEAS_SIZE, STATE_ORDER>& H,
                        EkfState& X                                             
    ) {
        // State update
        Eigen::Matrix<real, STATE_ORDER, 1> state_update = K * Y;
        X.pos += state_update.head<3>(); // Position update
        X.vel += state_update.block<3, 1>(S_VX, 0);
        X.gyro += state_update.block<3, 1>(S_ROLL_RATE, 0);
        X.acc += state_update.block<3, 1>(S_AX, 0);
        X.bg += state_update.block<3, 1>(S_B_ROLL_RATE, 0);
        X.ba += state_update.block<3, 1>(S_B_AX, 0);
        X.grav += state_update.block<3, 1>(S_G_X, 0);

        // Quaternion to rotation update
        Vector3 rot_delta = state_update.segment<3>(3);
        Quaternion quat_delta(AngleAxis(rot_delta.norm(), rot_delta.normalized()));
        X.rot = (X.rot * quat_delta).normalized();

        // Covariance update
        P = P - K * H * P;
    }

} // namespace InertialPoseLib
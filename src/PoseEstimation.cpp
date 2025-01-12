#include "InertialPoseLib/PoseEstimation.hpp"

namespace InertialPoseLib {
    void PoseEstimation::Reset(double imu_bias_std_gyro, double imu_bias_std_acc, bool estimate_gravity){
        
        imu_bias_std_gyro_ = imu_bias_std_gyro;
        imu_bias_std_acc_ = imu_bias_std_acc;
        estimate_gravity_ = estimate_gravity;

        // State initialization using configuration
        S_.pos.setZero();
        S_.rot.setIdentity();
        S_.vel.setZero();
        S_.gyro.setZero();
        S_.acc.setZero();
        S_.bg.setZero();
        S_.ba.setZero();
        S_.grav = Eigen::Vector3d(0.0, 0.0, imu_gravity_);

        // Covariance initialization using configuration
        P_ = Eigen::MatrixXd::Identity(STATE_ORDER, STATE_ORDER) * INIT_STATE_COV;

        P_(S_ROLL_RATE, S_ROLL_RATE) = imu_std_gyro_;
        P_(S_PITCH_RATE, S_PITCH_RATE) = imu_std_gyro_;
        P_(S_YAW_RATE, S_YAW_RATE) = imu_std_gyro_;

        P_(S_B_ROLL_RATE, S_B_ROLL_RATE) = imu_bias_std_gyro_;
        P_(S_B_PITCH_RATE, S_B_PITCH_RATE) = imu_bias_std_gyro_;
        P_(S_B_YAW_RATE, S_B_YAW_RATE) = imu_bias_std_gyro_;
        P_(S_B_AX, S_B_AX) = imu_bias_std_acc_;
        P_(S_B_AY, S_B_AY) = imu_bias_std_acc_;
        P_(S_B_AZ, S_B_AZ) = imu_bias_std_acc_;

        P_(S_G_X, S_G_X) = imu_bias_std_acc_;
        P_(S_G_Y, S_G_Y) = imu_bias_std_acc_;
        P_(S_G_Z, S_G_Z) = imu_bias_std_acc_;

        P_(S_IMU_ROLL, S_IMU_ROLL) = imu_bias_std_gyro_;
        P_(S_IMU_PITCH, S_IMU_PITCH) = imu_bias_std_gyro_;
        P_(S_IMU_YAW, S_IMU_YAW) = imu_bias_std_gyro_;

        prev_timestamp_ = 0.0;
        b_reset_for_init_prediction_ = true;

    }

    void PoseEstimation::PredictImu(const ImuStruct& imu_input){
        std::unique_lock<std::mutex> lock(mutex_state_, std::try_to_lock);

        if (lock.owns_lock() == false) {
            // If the lock is not owned, do not perform prediction (other process is accessing state)
            // Measurement Update priority
            return;
        }
        
        double cur_timestamp = imu_input.timestamp;

        if (b_reset_for_init_prediction_ == true) {
            prev_timestamp_ = cur_timestamp;

            b_reset_for_init_prediction_ = false;

            return;
        }

        // CheckRotationStabilized();

        // if (IsStateInitialized() == false) {
        //     prev_timestamp_ = cur_timestamp;

        //     // Complementary Filter requires Yaw initialization to estimate Roll and Pitch
        //     if (IsYawInitialized() == true &&
        //         (cfg_.i_gps_type == GpsType::BESTPOS || cfg_.b_use_complementary_filter)) {
        //         ComplementaryKalmanFilter(imu_input);
        //     }

        //     return false;
        // }

        // Calculate prediction dt
        double d_dt = cur_timestamp - prev_timestamp_;

        if (fabs(d_dt) < 1e-6) {
            return; // Not new data
        }

        EkfState ekf_state_prev = S_;

        // State rotation quat to matrix
        Eigen::Matrix3d G_R_I = S_.rot.toRotationMatrix();

        // Gyroscope reading corrected for bias
        Eigen::Vector3d corrected_gyro = imu_input.gyro - ekf_state_prev.bg;
        Eigen::Quaterniond delta_rot = ExpGyroToQuat(corrected_gyro, d_dt);
        S_.rot = (ekf_state_prev.rot * delta_rot).normalized();

        // Compensate IMU Bias and Gravity
        Eigen::Vector3d corrected_accel = imu_input.acc - ekf_state_prev.ba;
        Eigen::Vector3d accel_global = G_R_I * corrected_accel - ekf_state_prev.grav;

        // Predict Pose, Velocity
        S_.pos += ekf_state_prev.vel * d_dt + 0.5 * accel_global * d_dt * d_dt;
        S_.vel += accel_global * d_dt;

        // Use prior state of gyro and acc
        S_.gyro = corrected_gyro;
        S_.acc = accel_global;

        // Use prior state of bias and gravity
        S_.bg = ekf_state_prev.bg;
        S_.ba = ekf_state_prev.ba;
        S_.grav = ekf_state_prev.grav;

        // Process model matrix (Q) generation
        Eigen::Matrix<double, STATE_ORDER, STATE_ORDER> Q = Eigen::Matrix<double, STATE_ORDER, STATE_ORDER>::Zero();

        // Allocate std in Q matrix
        Q.block<3, 3>(S_X, S_X) = Eigen::Matrix3d::Identity() * std::pow(state_std_pos_m_, 2) * d_dt * d_dt;
        Q.block<3, 3>(S_ROLL, S_ROLL) =
                Eigen::Matrix3d::Identity() * std::pow(state_std_rot_rad_, 2) * d_dt * d_dt;
        Q.block<3, 3>(S_VX, S_VX) = Eigen::Matrix3d::Identity() * std::pow(state_std_vel_mps_, 2) * d_dt * d_dt;
        Q.block<3, 3>(S_ROLL_RATE, S_ROLL_RATE) =
                Eigen::Matrix3d::Identity() * std::pow(imu_std_gyro_, 2) * d_dt * d_dt;
        Q.block<3, 3>(S_AX, S_AX) = Eigen::Matrix3d::Identity() * std::pow(imu_std_acc_, 2) * d_dt * d_dt;
        Q.block<3, 3>(S_B_ROLL_RATE, S_B_ROLL_RATE) =
                Eigen::Matrix3d::Identity() * std::pow(imu_bias_std_gyro_, 2) * d_dt * d_dt;
        Q.block<3, 3>(S_B_AX, S_B_AX) =
                Eigen::Matrix3d::Identity() * std::pow(imu_bias_std_acc_, 2) * d_dt * d_dt;
        Q.block<3, 3>(S_G_X, S_G_X) = Eigen::Matrix3d::Identity() * std::pow(imu_bias_std_acc_, 2) * d_dt * d_dt;
        Q.block<3, 3>(S_IMU_ROLL, S_IMU_ROLL) =
                Eigen::Matrix3d::Identity() * std::pow(state_std_rot_rad_, 2) * d_dt * d_dt;

        // Covariance (P) propagation using CV model
        Eigen::Matrix<double, STATE_ORDER, STATE_ORDER> F = Eigen::Matrix<double, STATE_ORDER, STATE_ORDER>::Identity();

        // Position partial differentiation
        F.block<3, 3>(S_X, S_VX) = Eigen::Matrix3d::Identity() * d_dt; // ∂pos / ∂vel
        F.block<3, 3>(S_X, S_B_AX) = -0.5 * G_R_I * d_dt * d_dt; // ∂pos / ∂ba (including global transform)

        // Rotation partial differentiation
        F.block<3, 3>(S_ROLL, S_B_ROLL_RATE) = -PartialDerivativeRotWrtGyro(corrected_gyro, d_dt); // d rot / d bg

        // Velocity partial differentiation
        F.block<3, 3>(S_VX, S_B_AX) = -G_R_I * d_dt; // ∂vel / ∂ba (including global transform)
        F.block<3, 3>(S_ROLL_RATE, S_B_ROLL_RATE) = -Eigen::Matrix3d::Identity(); // ∂gyro / ∂bg
        F.block<3, 3>(S_AX, S_B_AX) = -G_R_I; // ∂acc / ∂ba (including global transform)

        if (estimate_gravity_) {
            // Only Z axis
            F(S_Z, S_G_Z) = -0.5 * d_dt * d_dt; // ∂z / ∂gz
            F(S_VZ, S_G_Z) = -d_dt;             // ∂vz / ∂gz
            F(S_AZ, S_G_Z) = -1.0;              // ∂az / ∂gz
        }

        // Covariance matrix
        P_ = F * P_ * F.transpose() + Q;

        prev_timestamp_ = cur_timestamp;

        std::cout<<"P_ROLL "<<P_(S_ROLL,S_ROLL) <<" P_ROLL_RATE "<<P_(S_ROLL_RATE,S_ROLL_RATE) 
         <<" P_B_ROLL_RATE "<<P_(S_B_ROLL_RATE,S_B_ROLL_RATE)<<std::endl;

        // TODO:
        // if (cfg_.b_use_zupt) ZuptImu(imu_input);
        // if (cfg_.i_gps_type == GpsType::BESTPOS || cfg_.b_use_complementary_filter) ComplementaryKalmanFilter(imu_input);

        ComplementaryKalmanFilter(imu_input);

        return;
    }

    void PoseEstimation::UpdateWithGnss(const GnssStruct& gnss_input, bool is_3dof){
        std::lock_guard<std::mutex> lock(mutex_state_);

        // CheckYawInitialized();
        // CheckStateInitialized();
        // CheckRotationStabilized();
        // CheckStateStabilized();

        // Main Algorithm

        // Observation matrix: H
        Eigen::Matrix<double, GNSS_MEAS_ORDER, STATE_ORDER> H;
        H.setZero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // Position
        H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity(); // Rotation

        // Measurement model: Z_state (state position and rotation)
        Eigen::Matrix<double, GNSS_MEAS_ORDER, 1> Z_state;
        Z_state.head<3>() = S_.pos; // Position

        // Measurement vector: Z (from gnss_input)
        Eigen::Matrix<double, GNSS_MEAS_ORDER, 1> Z;
        Z.head<3>() = gnss_input.pos;

        // Measurement covariance matrix: R
        Eigen::Matrix<double, GNSS_MEAS_ORDER, GNSS_MEAS_ORDER> R =
                Eigen::Matrix<double, GNSS_MEAS_ORDER, GNSS_MEAS_ORDER>::Zero();
        R.block<3, 3>(0, 0) = gnss_input.pos_cov;
        R.block<3, 3>(3, 3) = gnss_input.rot_cov;

        // Observation covariance matrix: S (GNSS_MEAS_ORDER, GNSS_MEAS_ORDER)
        Eigen::Matrix<double, GNSS_MEAS_ORDER, GNSS_MEAS_ORDER> S = H * P_ * H.transpose() + R;

        // Kalman Gain: K (STATE_ORDER * GNSS_MEAS_ORDER)
        Eigen::Matrix<double, STATE_ORDER, GNSS_MEAS_ORDER> K = P_ * H.transpose() * S.inverse();

        // Residual: Y (for position and rotation in quaternion)
        Eigen::Vector3d res_angle_euler = CalEulerResidualFromQuat(S_.rot, gnss_input.rot);

        Eigen::Matrix<double, GNSS_MEAS_ORDER, 1> Y;
        Y.head<3>() = Z.head<3>() - Z_state.head<3>(); // Position residual
        Y.tail<3>() = res_angle_euler;                 // Rotation residual as angle-axis vector


        if (is_3dof == true) {
            Eigen::Matrix<double, 3, STATE_ORDER> H3 = H.block<3, STATE_ORDER>(0, 0);
            Eigen::Matrix<double, 3, 3> S3 = H3 * P_ * H3.transpose() + R.block<3, 3>(0, 0);
            Eigen::Matrix<double, STATE_ORDER, 3> K3 = P_ * H3.transpose() * S3.inverse();
            Eigen::Matrix<double, 3, 1> Y3 = Y.head<3>();

            UpdateEkfState(K3, Y3, P_, H3, S_);
        }
        else {
            UpdateEkfState(K, Y, P_, H, S_);
        }


        return;
    }

    void PoseEstimation::PrintState() const{
        
        std::cout.precision(3);
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
        const Eigen::Vector3d vec_acc_meas = imu_input.acc - S_.ba; // Bias-corrected acceleration

        // Compensate centrifugal force
        // 1. Calculate speed in vehicle local frame
        Eigen::Vector3d vel_local = S_.rot.inverse() * S_.vel;

        // 4. Pure gravity component compensated for centrifugal force
        // Eigen::Vector3d compensated_acc = vec_acc_meas - vec_acc_centrip;
        Eigen::Vector3d compensated_acc = vec_acc_meas;

        // Check the magnitude of the compensated acceleration
        const double d_acc_sensor_magnitude = vec_acc_meas.norm();
        const double d_gravity_magnitude = S_.grav.norm();
        const double d_acc_diff = d_acc_sensor_magnitude - d_gravity_magnitude;

        // 1. Calculate measurement value (z) - use compensated acceleration for centrifugal force
        Eigen::Vector3d gravity_direction = compensated_acc.normalized();
        Eigen::Vector2d z;                                             // roll, pitch measurement value
        z << std::atan2(gravity_direction.y(), gravity_direction.z()), // roll
                -std::asin(gravity_direction.x());                     // pitch

        // 2. Extract roll, pitch of current state (h(x))
        Eigen::Vector3d current_rpy = RotToVec(S_.rot.toRotationMatrix());
        Eigen::Vector2d h_x;     // roll, pitch of current state
        h_x << current_rpy.x(),  // roll
                current_rpy.y(); // pitch

        // 3. Calculate Innovation (residual Y)
        Eigen::Vector2d innovation = z - h_x;

        // Normalize roll, pitch to the range [-π, π]
        innovation(0) = NormAngleRad(innovation(0));
        innovation(1) = NormAngleRad(innovation(1));

        // 4. Calculate Measurement Jacobian (H)
        Eigen::Matrix<double, 2, STATE_ORDER> H = Eigen::Matrix<double, 2, STATE_ORDER>::Zero();
        H(0, S_ROLL) = 1.0;  // ∂roll_meas/∂roll
        H(1, S_PITCH) = 1.0; // ∂pitch_meas/∂pitch

        // 5. Set Measurement noise covariance (R)
        Eigen::Matrix2d R = Eigen::Matrix2d::Zero();

        // Final measurement noise covariance
        R(0, 0) = 1.0;   // roll
        R(1, 1) = 1.0; // pitch

        // 6. Kalman gain
        Eigen::Matrix2d S = H * P_ * H.transpose() + R;                             // Innovation covariance
        Eigen::Matrix<double, STATE_ORDER, 2> K = P_ * H.transpose() * S.inverse(); // Kalman gain

        // 7. EKF State Update
        UpdateEkfState<2, 2>(K, innovation, P_, H, S_);

    }

    template <int MEAS_SIZE, int K_COLS>
    void PoseEstimation::UpdateEkfState(const Eigen::Matrix<double, STATE_ORDER, K_COLS>& K,
                        const Eigen::Matrix<double, MEAS_SIZE, 1>& Y,
                        Eigen::Matrix<double, STATE_ORDER, STATE_ORDER>& P,
                        const Eigen::Matrix<double, MEAS_SIZE, STATE_ORDER>& H,
                        EkfState& X                                             
    ) {
        // State update
        Eigen::Matrix<double, STATE_ORDER, 1> state_update = K * Y;
        X.pos += state_update.head<3>(); // Position update
        X.vel += state_update.block<3, 1>(S_VX, 0);
        X.gyro += state_update.block<3, 1>(S_ROLL_RATE, 0);
        X.acc += state_update.block<3, 1>(S_AX, 0);
        X.bg += state_update.block<3, 1>(S_B_ROLL_RATE, 0);
        X.ba += state_update.block<3, 1>(S_B_AX, 0);
        X.grav += state_update.block<3, 1>(S_G_X, 0);

        // Quaternion to rotation update
        Eigen::Vector3d rot_delta = state_update.segment<3>(3);
        Eigen::Quaterniond quat_delta(Eigen::AngleAxisd(rot_delta.norm(), rot_delta.normalized()));
        X.rot = (X.rot * quat_delta).normalized();

        // Quaternion to imu rotation
        Eigen::Vector3d imu_rot_delta = state_update.segment<3>(24);
        Eigen::Quaterniond imu_quat_delta(Eigen::AngleAxisd(imu_rot_delta.norm(), imu_rot_delta.normalized()));
        X.imu_rot = (X.imu_rot * imu_quat_delta).normalized();

        // Covariance update
        P = P - K * H * P;
    }

} // namespace InertialPoseLib
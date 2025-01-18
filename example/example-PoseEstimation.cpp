#include <iostream>
#include <InertialPoseLib/PoseEstimation.hpp>

int main() {
    InertialPoseLib::PoseEstimationParams pose_estimation_params = {
        0.0001,  // imu_gyro_std
        0.001,   // imu_acc_std
        0.0001,  // imu_bias_gyro_std
        0.0001,  // imu_bias_acc_std

        9.81,

        0.02,    // state_std_pos_m
        0.003,    // state_std_rot_rad
        2.0,    // state_std_vel_mps

        false,   // estimate_imu_bias
        false    // estimate_gravity
    };
    
    double timestamp = 0.0; 
    int max_sim_count = 100;

    InertialPoseLib::PoseEstimation pose_estimation(pose_estimation_params);
    
    int count = 0;
    while (count < max_sim_count) {
        std::cout<<"Test inertial_pose_test_pkg"<<std::endl;
        
        InertialPoseLib::ImuStruct imu_input;
        imu_input.timestamp = timestamp;
        imu_input.acc.z() = 9.8;
        imu_input.gyro.x() = 0.1;

        pose_estimation.PredictImu(imu_input);


        InertialPoseLib::GnssStruct gnss_input;
        gnss_input.timestamp = timestamp;
        gnss_input.pos.x() = 1.0;
        gnss_input.pos.y() = 2.0;
        gnss_input.pos_cov = Eigen::Matrix3d::Identity() * 0.1;

        gnss_input.rot = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
        gnss_input.rot_cov = Eigen::Matrix3d::Identity() * 0.1;

        pose_estimation.UpdateWithGnss(gnss_input);

        pose_estimation.PrintState();

        InertialPoseLib::EkfState ekf_state = pose_estimation.GetCurrentState();

        timestamp += 0.1;
    }
}
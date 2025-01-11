#include <iostream>
#include <InertialPoseLib/PoseEstimation.hpp>

int main() {
    InertialPoseLib::PoseEstimation pose_estimation(0.001, 0.001);
    
    double timestamp = 0.0; 
    int max_sim_count = 100;

    InertialPoseLib::PoseEstimation pose_estimation(0.001, 0.001);
    
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

        timestamp += 0.1;
    }
}
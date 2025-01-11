#ifndef __UTILS_HPP__
#define __UTILS_HPP__
#pragma once

#include "InertialPoseLib/DataType.hpp"

namespace InertialPoseLib {
    /**
     * Calculate interpolated transformation between two transformation matrices
     * @param affine_trans_between start and end transformation
     * @param dt_scan current time and start time difference
     * @param dt_trans end time and start time difference
     * @return interpolated transformation matrix
     */
    inline Eigen::Affine3f InterpolateTfWithTime(const Eigen::Affine3f& affine_trans_between, double dt_scan, double dt_trans) {
        // Check if dt_trans is zero to avoid division by zero
        if (dt_trans == 0.0) {
            return Eigen::Affine3f::Identity();
        }

        // Compute the interpolation ratio
        double ratio = dt_scan / dt_trans;

        // Interpolate translation part
        Eigen::Vector3f translation = affine_trans_between.translation() * ratio;

        // Interpolate rotation part using quaternion slerp (for smooth interpolation)
        Eigen::Quaternionf rotation(affine_trans_between.rotation());
        Eigen::Quaternionf interpolatedRotation = Eigen::Quaternionf::Identity().slerp(ratio, rotation);

        // Combine translation and rotation to create the final transformation
        Eigen::Affine3f affine_interpolated_transform = Eigen::Affine3f::Identity();
        affine_interpolated_transform.translate(translation);
        affine_interpolated_transform.rotate(interpolatedRotation);

        return affine_interpolated_transform;
    }

    /**
     * Normalize angle to 0~360 degrees
     * @param d_angle_deg input angle (degrees)
     * @return normalized angle (degrees)
     */
    inline double NormAngleDeg(double d_angle_deg) {
        double d_angle_norm_deg = d_angle_deg;

        // Set the input angle into the 0~pi
        while (d_angle_norm_deg > 360.) d_angle_norm_deg -= 360.;
        while (d_angle_norm_deg < 0.) d_angle_norm_deg += 360.;

        return d_angle_norm_deg;
    }

    /**
     * Normalize angle to -π~π radians
     * @param d_angle_rad input angle (radians)
     * @return normalized angle (radians)
     */
    inline double NormAngleRad(double d_angle_rad) {
        double d_angle_norm_rad = d_angle_rad;

        // Set the input angle into the 0~pi
        while (d_angle_norm_rad > M_PI) d_angle_norm_rad -= M_PI * 2.;
        while (d_angle_norm_rad < -M_PI) d_angle_norm_rad += M_PI * 2.;

        return d_angle_norm_rad;
    }

    /**
     * Calculate angle difference (-180~180 degrees)
     * @param d_ref_deg reference angle (degrees)
     * @param d_rel_deg relative angle (degrees)
     * @return angle difference (degrees)
     */
    inline double AngleDiffDeg(double d_ref_deg, double d_rel_deg) {
        double d_angle_diff_deg = d_rel_deg - d_ref_deg;

        // calculate angle difference
        while (d_angle_diff_deg > 180.) d_angle_diff_deg = d_angle_diff_deg - 360.;
        while (d_angle_diff_deg < -180.) d_angle_diff_deg = d_angle_diff_deg + 360.;

        return d_angle_diff_deg;
    }

    /**
     * Calculate angle difference (-π~π radians)
     * @param d_ref_rad reference angle (radians)
     * @param d_rel_rad relative angle (radians)
     * @return angle difference (radians)
     */
    inline double AngleDiffRad(double d_ref_rad, double d_rel_rad) {
        double d_angle_diff_rad = d_rel_rad - d_ref_rad;

        // calculate angle difference
        while (d_angle_diff_rad > M_PI) d_angle_diff_rad = d_angle_diff_rad - 2. * M_PI;
        while (d_angle_diff_rad < -M_PI) d_angle_diff_rad = d_angle_diff_rad + 2. * M_PI;

        return d_angle_diff_rad;
    }


    // Rotation Matrix to Euler angles (to avoid gimbal lock)
    /**
     * Convert rotation matrix to Euler angles (to avoid gimbal lock)
     * @param R 3x3 rotation matrix
     * @return 3D vector containing roll, pitch, yaw
     */
    inline Eigen::Vector3d RotToVec(const Eigen::Matrix3d& R) {
        Eigen::Vector3d angles;

        // Special case handling (to detect gimbal lock)
        if (std::abs(R(2, 0)) > 0.998) { // gimbal lock occurs
            angles(2) = std::atan2(-R(1, 2), R(1, 1));
            angles(1) = M_PI / 2 * (R(2, 0) >= 0 ? 1 : -1);
            angles(0) = 0;
        }
        else {
            angles(1) = std::asin(-R(2, 0));
            angles(0) = std::atan2(R(2, 1) / std::cos(angles(1)), R(2, 2) / std::cos(angles(1)));
            angles(2) = std::atan2(R(1, 0) / std::cos(angles(1)), R(0, 0) / std::cos(angles(1)));
        }

        // Normalize angles to be within -π and π
        angles(0) = std::fmod(angles(0) + M_PI, 2 * M_PI) - M_PI;
        angles(1) = std::fmod(angles(1) + M_PI, 2 * M_PI) - M_PI;
        angles(2) = std::fmod(angles(2) + M_PI, 2 * M_PI) - M_PI;

        return angles;
    }

    /**
     * Convert Euler angles to rotation matrix
     * @param angles 3D vector containing roll, pitch, yaw
     * @return 3x3 rotation matrix
     */
    inline Eigen::Matrix3d VecToRot(const Eigen::Vector3d& angles) {
        Eigen::Matrix3d R = (Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
        return R;
    }


    // Calculate angle difference between two quaternions
    /**
     * Calculate angle difference between two quaternions
     * @param state_quat state quaternion
     * @param measurement_quat measurement quaternion
     * @return angle difference (radians)
     */
    inline Eigen::Vector3d CalEulerResidualFromQuat(const Eigen::Quaterniond& state_quat,
                                                    const Eigen::Quaterniond& measurement_quat) {
        // Normalize quaternions to rotation matrices
        Eigen::Vector3d state_angles = RotToVec(state_quat.normalized().toRotationMatrix());
        Eigen::Vector3d meas_angles = RotToVec(measurement_quat.normalized().toRotationMatrix());

        // Calculate Euler angle residual
        Eigen::Vector3d res_euler = meas_angles - state_angles;

        // Normalize angles to be within -π and π
        res_euler.x() = NormAngleRad(res_euler.x());
        res_euler.y() = NormAngleRad(res_euler.y());
        res_euler.z() = NormAngleRad(res_euler.z());

        return res_euler;
    }

} // namespace InertialPoseLib

#endif
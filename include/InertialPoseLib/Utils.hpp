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
    inline Affine3 InterpolateTfWithTime(const Affine3& affine_trans_between, real dt_scan, real dt_trans) {
        // Check if dt_trans is zero to avoid division by zero
        if (dt_trans == 0.0) {
            return Affine3::Identity();
        }

        // Compute the interpolation ratio
        real ratio = dt_scan / dt_trans;

        // Interpolate translation part
        Vector3 translation = affine_trans_between.translation() * ratio;

        // Interpolate rotation part using quaternion slerp (for smooth interpolation)
        Quaternion rotation(affine_trans_between.rotation());
        Quaternion interpolatedRotation = Quaternion::Identity().slerp(ratio, rotation);

        // Combine translation and rotation to create the final transformation
        Affine3 affine_interpolated_transform = Affine3::Identity();
        affine_interpolated_transform.translate(translation);
        affine_interpolated_transform.rotate(interpolatedRotation);

        return affine_interpolated_transform;
    }

    /**
     * Normalize angle to 0~360 degrees
     * @param d_angle_deg input angle (degrees)
     * @return normalized angle (degrees)
     */
    inline real NormAngleDeg(real d_angle_deg) {
        real d_angle_norm_deg = d_angle_deg;

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
    inline real NormAngleRad(real d_angle_rad) {
        real d_angle_norm_rad = d_angle_rad;

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
    inline real AngleDiffDeg(real d_ref_deg, real d_rel_deg) {
        real d_angle_diff_deg = d_rel_deg - d_ref_deg;

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
    inline real AngleDiffRad(real d_ref_rad, real d_rel_rad) {
        real d_angle_diff_rad = d_rel_rad - d_ref_rad;

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
    inline Vector3 RotToVec(const Matrix3& R) {
        Vector3 angles;

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
    inline Matrix3 VecToRot(const Vector3& angles) {
        Matrix3 R = (AngleAxis(angles.z(), Vector3::UnitZ()) *
                            AngleAxis(angles.y(), Vector3::UnitY()) *
                            AngleAxis(angles.x(), Vector3::UnitX())).toRotationMatrix();
        return R;
    }


    // Calculate angle difference between two quaternions
    /**
     * Calculate angle difference between two quaternions
     * @param state_quat state quaternion
     * @param measurement_quat measurement quaternion
     * @return angle difference (radians)
     */
    inline Vector3 CalEulerResidualFromQuat(const Quaternion& state_quat,
                                                    const Quaternion& measurement_quat) {
        // Normalize quaternions to rotation matrices
        Vector3 state_angles = RotToVec(state_quat.normalized().toRotationMatrix());
        Vector3 meas_angles = RotToVec(measurement_quat.normalized().toRotationMatrix());

        // Calculate Euler angle residual
        Vector3 res_euler = meas_angles - state_angles;

        // Normalize angles to be within -π and π
        res_euler.x() = NormAngleRad(res_euler.x());
        res_euler.y() = NormAngleRad(res_euler.y());
        res_euler.z() = NormAngleRad(res_euler.z());

        return res_euler;
    }

} // namespace InertialPoseLib

#endif
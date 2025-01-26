/**
 * @file LinearAlgebra.hpp
 * @author Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Linear Algebra functions for InertialPoseLib
 * @version 0.1
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025
 */

#ifndef __LINEAR_ALGEBRA_HPP__
#define __LINEAR_ALGEBRA_HPP__
#pragma once

// STD header
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Libraries
#include <Eigen/Dense>
#include "InertialPoseLib/DataType.hpp"

namespace InertialPoseLib {
    /*
        Lie Algebra
    */
    /**
     * Convert vector to skew-symmetric matrix
     * @param V 3D vector
     * @return 3x3 skew-symmetric matrix
     */
    inline Matrix3 SkewSymmetricMatrix(const Vector3& V) {
        Matrix3 M;

        M << 0.0, -V(2), V(1), V(2), 0.0, -V(0), -V(1), V(0), 0.0;
        return M;
    }

    // SO(3) non-linear space --> so(3) linear space 
    /**
     * SO(3) non-linear space --> so(3) linear space (log mapping)
     * @param R SO(3) rotation matrix
     * @return so(3) 3D vector
     */
    inline Vector3 Log(const Matrix3& R) {
        Real cos_theta = (R.trace() - 1.0) * 0.5;
        cos_theta = std::min(Real(1.0), std::max(Real(-1.0), cos_theta));  // Clamping for numerical stability
        Real theta = std::acos(cos_theta);
        
        if (std::abs(theta) < 1e-5) {
            return Vector3::Zero();
        }
        Matrix3 log_matrix = (R - R.transpose()) / (2.0 * std::sin(theta));
        return theta * Vector3(log_matrix(2, 1), log_matrix(0, 2), log_matrix(1, 0));
    }

    /**
     * so(3) linear space --> SO(3) non-linear space (exp mapping)
     * @param omega so(3) 3D vector
     * @return SO(3) rotation matrix
     */
    inline Matrix3 Exp(const Vector3& omega) {
        Real theta = omega.norm();
        Matrix3 Eye3 = Matrix3::Identity();
        if (theta < 1e-5) {
            return Eye3;
        }
        Vector3 axis = omega / theta;
        Matrix3 K = SkewSymmetricMatrix(axis);
        return Eye3 + std::sin(theta) * K + (1 - std::cos(theta)) * K * K;
    }

    /**
     * Calculate rotation matrix change using angular velocity and time
     * @param gyro angular velocity vector
     * @param d_dt_sec time step
     * @return rotation matrix change
     */
    inline Matrix3 ExpGyroToRotMatrix(const Vector3& gyro, Real d_dt_sec) {
        Vector3 omega = gyro * d_dt_sec; // Angular velocity scaled by time step
        return Exp(omega); // Use the ExpMap function to get the rotation matrix
    }


    /**
     * Calculate quaternion change using angular velocity and time
     * @param gyro angular velocity vector
     * @param d_dt_sec time step
     * @return quaternion change
     */
    inline Quaternion ExpGyroToQuat(const Vector3& gyro, Real d_dt_sec) {
        Vector3 omega = gyro * d_dt_sec; // Angular velocity vector scaled by time step
        Matrix3 rotation_matrix = Exp(omega); // Use the Exp function
        return Quaternion(rotation_matrix); // Convert rotation matrix to quaternion
    }

    /*
        Given:
        δ_rot = Exp(gyro * Δt)
        ∂δ_rot / ∂ω = ∂Exp(ω) / ∂ω = ∂/∂ω (I + sin(θ) * K + (1 - cos(θ)) * K^2)
        ω = gyro * Δt
        θ = ||ω||

        The partial derivative of δ_rot with respect to gyro:
        
        ∂δ_rot / ∂gyro ≈ Δt * (I + (1 - cos(θ)) / θ^2 * K + (θ - sin(θ)) / θ^3 * K^2)

        where:
        - I is the identity matrix
        - K is the skew-symmetric matrix of ω/θ
    */
    /**
     * Calculate partial derivative of rotation with respect to angular velocity
     * @param gyro angular velocity vector
     * @param d_dt_sec time step
     * @return 3x3 Jacobian matrix
     */
    inline Matrix3 PartialDerivativeRotWrtGyro(const Vector3& gyro, Real d_dt_sec) {

        Vector3 omega = gyro * d_dt_sec; // angular velocity vector scaled by time step
        Real theta = omega.norm(); // total angular velocity

        if (theta < 1e-5) {
            return Matrix3::Zero(); // Near-zero rotation, derivative is approximately zero
        }

        Vector3 axis = omega / theta; // rotation axis = angular velocity vector / total angular velocity
        Matrix3 K = SkewSymmetricMatrix(axis); // skew-symmetric matrix: rotation vector representation in SO(3)
        Matrix3 partial_derivative = d_dt_sec * 
                                            (Matrix3::Identity() 
                                            + (1 - std::cos(theta)) / (theta * theta) * K 
                                            + (theta - std::sin(theta)) / (theta * theta * theta) * K * K);

        return partial_derivative;
    }

} // namespace InertialPoseLib

#endif
#pragma once

#include <algorithm>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include "loop_closure/loop_closure_types.hpp"

namespace uwfl2::loop_closure
{

using Matrix27d = Eigen::Matrix<double, 27, 27>;

inline Matrix27d correction_transport_jacobian(
    const Eigen::Matrix3d &correction_rotation)
{
    Matrix27d jacobian = Matrix27d::Identity();
    jacobian.block<3, 3>(0, 0) = correction_rotation;
    // Right attitude errors are invariant to a deterministic left correction.
    jacobian.block<3, 3>(3, 3).setIdentity();
    jacobian.block<3, 3>(12, 12) = correction_rotation;
    return jacobian;
}

inline Matrix27d transport_uwfl2_covariance(
    const Matrix27d &covariance,
    const Eigen::Matrix3d &correction_rotation)
{
    const Matrix27d jacobian =
        correction_transport_jacobian(correction_rotation);
    Matrix27d transported = jacobian * covariance * jacobian.transpose();
    return 0.5 * (transported + transported.transpose());
}

// Treat the accepted graph correction plus latest-scan registration as a pose
// pseudo-measurement. The Joseph update preserves PSD and updates every state
// only through its existing cross-covariance with the six pose coordinates.
inline Matrix27d apply_pose_covariance_update(
    const Matrix27d &transported_covariance,
    const Matrix6d &pose_measurement_covariance_position_rotation)
{
    Matrix6d measurement_covariance =
        0.5 * (pose_measurement_covariance_position_rotation +
               pose_measurement_covariance_position_rotation.transpose());
    if (!transported_covariance.allFinite() ||
        !measurement_covariance.allFinite())
    {
        return Matrix27d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    Eigen::SelfAdjointEigenSolver<Matrix6d> noise_solver(measurement_covariance);
    if (noise_solver.info() != Eigen::Success ||
        noise_solver.eigenvalues().minCoeff() < -1e-12)
    {
        return Matrix27d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    measurement_covariance = noise_solver.eigenvectors() *
                             noise_solver.eigenvalues().cwiseMax(1e-12).asDiagonal() *
                             noise_solver.eigenvectors().transpose();

    Eigen::Matrix<double, 6, 27> H = Eigen::Matrix<double, 6, 27>::Zero();
    H.block<6, 6>(0, 0).setIdentity();
    const Matrix6d innovation_covariance =
        H * transported_covariance * H.transpose() + measurement_covariance;
    Eigen::LDLT<Matrix6d> innovation_solver(innovation_covariance);
    if (innovation_solver.info() != Eigen::Success)
    {
        return Matrix27d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    const Eigen::Matrix<double, 27, 6> gain =
        transported_covariance * H.transpose() *
        innovation_solver.solve(Matrix6d::Identity());
    const Matrix27d identity = Matrix27d::Identity();
    const Matrix27d update = identity - gain * H;
    Matrix27d posterior = update * transported_covariance * update.transpose() +
                          gain * measurement_covariance * gain.transpose();
    return 0.5 * (posterior + posterior.transpose());
}

inline bool covariance27_is_valid(const Matrix27d &covariance,
                                  double tolerance = 1e-9)
{
    if (!covariance.allFinite())
    {
        return false;
    }
    Eigen::SelfAdjointEigenSolver<Matrix27d> solver(
        0.5 * (covariance + covariance.transpose()),
        Eigen::EigenvaluesOnly);
    return solver.info() == Eigen::Success &&
           solver.eigenvalues().minCoeff() >= -tolerance;
}

}  // namespace uwfl2::loop_closure

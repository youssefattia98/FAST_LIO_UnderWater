#pragma once

#include <Eigen/Core>
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
    const Eigen::Matrix3d &correction_rotation,
    const Matrix6d &pose_correction_covariance_position_rotation)
{
    const Matrix27d jacobian =
        correction_transport_jacobian(correction_rotation);
    Matrix27d transported = jacobian * covariance * jacobian.transpose();
    transported.block<6, 6>(0, 0) +=
        pose_correction_covariance_position_rotation;
    return 0.5 * (transported + transported.transpose());
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

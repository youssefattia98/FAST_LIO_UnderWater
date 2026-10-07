#pragma once

#include <array>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace uwfl2
{

struct RosPoseCovariance
{
    std::array<double, 36> values{};
    bool valid = false;
    bool projected_to_psd = false;
    double minimum_eigenvalue = 0.0;
};

// IKFOM stores position error in the local/world frame and right attitude
// error in the body tangent. ROS pose covariance uses [x,y,z,rx,ry,rz] in
// the odometry header frame, so rotate the attitude error into that frame.
template <typename Covariance>
RosPoseCovariance make_ros_pose_covariance(
    const Covariance &ikf_covariance,
    const Eigen::Matrix3d &R_local_body)
{
    RosPoseCovariance output;
    if (!ikf_covariance.allFinite() || !R_local_body.allFinite())
    {
        return output;
    }

    Eigen::Matrix<double, 6, 6> transform =
        Eigen::Matrix<double, 6, 6>::Identity();
    transform.block<3, 3>(3, 3) = R_local_body;

    Eigen::Matrix<double, 6, 6> covariance =
        transform * ikf_covariance.template block<6, 6>(0, 0) *
        transform.transpose();
    covariance = 0.5 * (covariance + covariance.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(covariance);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
    {
        return output;
    }
    output.minimum_eigenvalue = solver.eigenvalues().minCoeff();
    if (output.minimum_eigenvalue < 0.0)
    {
        const auto eigenvalues = solver.eigenvalues().cwiseMax(0.0);
        covariance = solver.eigenvectors() * eigenvalues.asDiagonal() *
                     solver.eigenvectors().transpose();
        covariance = 0.5 * (covariance + covariance.transpose());
        output.projected_to_psd = true;
    }

    for (int row = 0; row < 6; ++row)
    {
        for (int column = 0; column < 6; ++column)
        {
            output.values[static_cast<std::size_t>(row * 6 + column)] =
                covariance(row, column);
        }
    }
    output.valid = true;
    return output;
}

}  // namespace uwfl2

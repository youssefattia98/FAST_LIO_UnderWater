#pragma once

#include <cmath>
#include <optional>
#include <vector>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace uwfl2
{
// Independent derivation of GenZ-LIO, arXiv:2603.16273v1, equations 16--30.
// Local <- body right-SO(3) perturbation; fixed body <- sonar extrinsics.
// UWFL2 tangent order starts with position[0:3], rotation[3:6]. All remaining
// 21 columns are zero, but the full covariance retains cross-state correction.
struct SonarResidualRow
{
    double residual = 0.0, variance = 0.0;
    Eigen::Matrix<double, 1, 27> jacobian = Eigen::Matrix<double, 1, 27>::Zero();
    double Innovation() const { return -residual / std::sqrt(variance); }
    Eigen::Matrix<double, 1, 27> WhitenedJacobian() const { return jacobian / std::sqrt(variance); }
};

struct SonarReferencePoint
{
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    // Measurement uncertainty conditional on the accepted map poses. Shared
    // pose error cannot be added here as independent per-point noise.
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
};

struct SonarReferencePlane
{
    Eigen::Vector3d normal = Eigen::Vector3d::Zero(), center = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
};

inline Eigen::Matrix3d SonarCartesianCovariance(const Eigen::Vector3d &point,
                                               const Eigen::Vector3d &spherical_variance)
{
    const double range = point.norm(), azimuth = std::atan2(point.y(), point.x());
    const double elevation = std::atan2(point.z(), std::hypot(point.x(), point.y()));
    const double ca = std::cos(azimuth), sa = std::sin(azimuth);
    const double ce = std::cos(elevation), se = std::sin(elevation);
    Eigen::Matrix3d derivative;
    derivative << ce * ca, -range * ce * sa, -range * se * ca,
                  ce * sa,  range * ce * ca, -range * se * sa,
                  se,      0.0,             range * ce;
    return derivative * spherical_variance.asDiagonal() * derivative.transpose();
}

// Covariance of [normal, center], including their correlation. The references
// carry independent measurement covariances in the same local frame. Map-pose
// and map/live-state cross-correlations are not represented: this is a
// conditional-map approximation, not an augmented-state map filter. PCA alone
// does not establish that this neighborhood is a trustworthy plane match.
inline std::optional<SonarReferencePlane> FitSonarReferencePlane(
    const std::vector<SonarReferencePoint> &points)
{
    if (points.size() < 3) return std::nullopt;
    SonarReferencePlane plane;
    for (const auto &point : points)
    {
        if (!point.position.allFinite() || !point.covariance.allFinite()) return std::nullopt;
        plane.center += point.position;
    }
    const double count = points.size();
    plane.center /= count;
    Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
    for (const auto &point : points)
    {
        const Eigen::Vector3d offset = point.position - plane.center;
        scatter += offset * offset.transpose() / count;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(scatter);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues()[1] - eigen.eigenvalues()[0] <=
        1e-10 * std::max(1.0, eigen.eigenvalues()[2])) return std::nullopt;
    plane.normal = eigen.eigenvectors().col(0);
    for (const auto &point : points)
    {
        const Eigen::Vector3d offset = point.position - plane.center;
        Eigen::Matrix<double, 6, 3> derivative = Eigen::Matrix<double, 6, 3>::Zero();
        for (int axis = 1; axis < 3; ++axis)
        {
            const Eigen::Vector3d tangent = eigen.eigenvectors().col(axis);
            derivative.topRows<3>() += tangent *
                (tangent.dot(offset) * plane.normal.transpose() + plane.normal.dot(offset) * tangent.transpose()) /
                (count * (eigen.eigenvalues()[0] - eigen.eigenvalues()[axis]));
        }
        derivative.bottomRows<3>() = Eigen::Matrix3d::Identity() / count;
        plane.covariance += derivative * point.covariance * derivative.transpose();
    }
    plane.covariance = (0.5 * (plane.covariance + plane.covariance.transpose())).eval();
    return plane;
}

inline SonarResidualRow MakeSonarRow(const Eigen::Vector3d &direction,
                                    const Eigen::Vector3d &point_body,
                                    const Eigen::Matrix3d &rotation_local_body)
{
    SonarResidualRow row;
    row.jacobian.head<3>() = direction.transpose();
    row.jacobian.segment<3>(3) = point_body.cross(rotation_local_body.transpose() * direction).transpose();
    return row;
}

inline std::optional<SonarResidualRow> SonarPlaneRow(
    const Eigen::Vector3d &point_body, const Eigen::Matrix3d &rotation_local_body,
    const Eigen::Vector3d &position_local, const Eigen::Matrix3d &query_covariance_local,
    const SonarReferencePlane &plane)
{
    const Eigen::Vector3d offset = rotation_local_body * point_body + position_local - plane.center;
    auto row = MakeSonarRow(plane.normal, point_body, rotation_local_body);
    row.residual = plane.normal.dot(offset);
    Eigen::Matrix<double, 1, 6> noise_jacobian;
    noise_jacobian << offset.transpose(), -plane.normal.transpose();
    row.variance = (noise_jacobian * plane.covariance * noise_jacobian.transpose())(0, 0) +
                   plane.normal.dot(query_covariance_local * plane.normal);
    if (!std::isfinite(row.variance) || row.variance <= 0.0 || !std::isfinite(row.residual) ||
        !row.jacobian.allFinite()) return std::nullopt;
    return row;
}

inline std::optional<SonarResidualRow> SonarPointRow(
    const Eigen::Vector3d &point_body, const Eigen::Matrix3d &rotation_local_body,
    const Eigen::Vector3d &position_local, const Eigen::Matrix3d &query_covariance_local,
    const SonarReferencePoint &reference, double root_voxel_size,
    std::size_t accessed_voxels, std::size_t evaluated_points, double metric_scale = 1.0)
{
    const Eigen::Vector3d offset = rotation_local_body * point_body + position_local - reference.position;
    const double distance = offset.norm();
    if (!std::isfinite(distance) || distance < 1e-8 || evaluated_points == 0 ||
        accessed_voxels == 0 || !std::isfinite(root_voxel_size) || root_voxel_size <= 0.0 ||
        !std::isfinite(metric_scale) || metric_scale <= 0.0) return std::nullopt;
    const Eigen::Vector3d direction = offset / distance;
    auto row = MakeSonarRow(direction, point_body, rotation_local_body);
    row.residual = distance;
    // GenZ equation 29: use actual bounded-search statistics, not leaf^2/12.
    const double discretization_variance = accessed_voxels * root_voxel_size * root_voxel_size / evaluated_points;
    row.variance = metric_scale * (direction.dot((query_covariance_local + reference.covariance) * direction) +
                                  discretization_variance);
    if (!std::isfinite(row.variance) || row.variance <= 0.0 || !row.jacobian.allFinite()) return std::nullopt;
    return row;
}
}  // namespace uwfl2

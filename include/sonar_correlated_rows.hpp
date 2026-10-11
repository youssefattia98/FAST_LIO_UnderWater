#pragma once

#include <array>
#include <map>
#include "sonar_registration_math.hpp"

namespace uwfl2
{
struct SonarCorrelatedRow
{
    SonarResidualRow measurement;
    std::optional<SonarReferencePlane> cached_plane;
    Eigen::Matrix<double, 1, 6> plane_noise_jacobian =
        Eigen::Matrix<double, 1, 6>::Zero();
};

struct SonarWhitenedSystem
{
    Eigen::MatrixXd jacobian;
    Eigen::VectorXd innovation;
    bool valid = false;
};

// Shared fitted-plane error gives R = D + U U^T within each cached patch.
// Apply its inverse square root through a six-dimensional eigensystem, rather
// than allocating a dense covariance over all scan returns. This is conditional
// on accepted map poses; it does not estimate map/live-state cross-covariance.
inline SonarWhitenedSystem WhitenSonarRows(const std::vector<SonarCorrelatedRow> &rows)
{
    SonarWhitenedSystem result;
    result.jacobian = Eigen::MatrixXd::Zero(rows.size(), 27);
    result.innovation = Eigen::VectorXd::Zero(rows.size());
    std::map<std::array<double, 6>, std::vector<std::size_t>> groups;
    for (std::size_t i = 0; i < rows.size(); ++i)
    {
        const auto &row = rows[i];
        if (!row.measurement.jacobian.allFinite() || !std::isfinite(row.measurement.residual) ||
            !std::isfinite(row.measurement.variance) || row.measurement.variance <= 0)
            return result;
        result.jacobian.row(i) = row.measurement.WhitenedJacobian();
        result.innovation[i] = row.measurement.Innovation();
        if (row.cached_plane)
        {
            const auto &plane = *row.cached_plane;
            if (!plane.normal.allFinite() || !plane.center.allFinite() ||
                !row.plane_noise_jacobian.allFinite()) return result;
            groups[{plane.normal.x(), plane.normal.y(), plane.normal.z(),
                    plane.center.x(), plane.center.y(), plane.center.z()}].push_back(i);
        }
    }
    for (const auto &group : groups)
    {
        const auto &indices = group.second;
        const auto &covariance = rows[indices.front()].cached_plane->covariance;
        if (!covariance.allFinite() ||
            (covariance - covariance.transpose()).norm() > 1e-10) return result;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> plane_eigen(covariance);
        if (plane_eigen.info() != Eigen::Success ||
            plane_eigen.eigenvalues().minCoeff() < -1e-10) return result;
        const Eigen::Matrix<double, 6, 6> root = plane_eigen.eigenvectors() *
            plane_eigen.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal();
        if (root.squaredNorm() == 0) continue;
        Eigen::MatrixXd low_rank(indices.size(), 6), values(indices.size(), 28);
        for (std::size_t j = 0; j < indices.size(); ++j)
        {
            const auto &row = rows[indices[j]];
            if (!row.cached_plane->covariance.isApprox(covariance, 1e-12)) return result;
            const Eigen::Matrix<double, 1, 6> factor = row.plane_noise_jacobian * root;
            const double independent_variance = row.measurement.variance - factor.squaredNorm();
            if (!std::isfinite(independent_variance) || independent_variance <= 0) return result;
            const double scale = 1 / std::sqrt(independent_variance);
            low_rank.row(j) = scale * factor;
            values.row(j).head<27>() = scale * row.measurement.jacobian;
            values(j, 27) = -scale * row.measurement.residual;
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(
            low_rank.transpose() * low_rank);
        if (eigen.info() != Eigen::Success) return result;
        // ((1+lambda)^(-1/2)-1)/lambda, with its stable lambda=0 limit.
        Eigen::Matrix<double, 6, 1> coefficients;
        for (int i = 0; i < 6; ++i)
        {
            const double scale = std::sqrt(1 + std::max(0.0, eigen.eigenvalues()[i]));
            coefficients[i] = -1 / (scale * (scale + 1));
        }
        const Eigen::MatrixXd basis = low_rank * eigen.eigenvectors();
        values += (basis * (coefficients.asDiagonal() * (basis.transpose() * values))).eval();
        if (!values.allFinite()) return result;
        for (std::size_t j = 0; j < indices.size(); ++j)
        {
            result.jacobian.row(indices[j]) = values.row(j).head<27>();
            result.innovation[indices[j]] = values(j, 27);
        }
    }
    result.valid = !rows.empty();
    return result;
}
}  // namespace uwfl2

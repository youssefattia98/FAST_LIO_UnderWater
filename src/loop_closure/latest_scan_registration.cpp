#include "loop_closure/latest_scan_registration.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Eigenvalues>

namespace uwfl2::loop_closure
{
namespace
{

using Clock = std::chrono::steady_clock;
constexpr int kNearestNeighbors = 5;

Eigen::Matrix3d skew(const Eigen::Vector3d &vector)
{
    Eigen::Matrix3d result;
    result << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(),
        -vector.y(), vector.x(), 0.0;
    return result;
}

Eigen::Quaterniond exp_so3(const Eigen::Vector3d &rotation)
{
    const double angle = rotation.norm();
    if (angle < 1e-12)
    {
        return Eigen::Quaterniond(
                   Eigen::Matrix3d::Identity() + skew(rotation))
            .normalized();
    }
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(angle, rotation / angle));
}

struct CorrespondenceSet
{
    std::vector<Eigen::Matrix<double, 1, 6>> jacobians;
    std::vector<double> residuals;
};

bool fit_plane(const ShadowPointVector &neighbors,
               double threshold,
               Eigen::Vector3d *normal,
               double *offset)
{
    if (neighbors.size() < kNearestNeighbors)
    {
        return false;
    }
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const ShadowPoint &point : neighbors)
    {
        centroid += Eigen::Vector3d(point.x, point.y, point.z);
    }
    centroid /= static_cast<double>(neighbors.size());
    Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
    for (const ShadowPoint &point : neighbors)
    {
        const Eigen::Vector3d centered =
            Eigen::Vector3d(point.x, point.y, point.z) - centroid;
        scatter.noalias() += centered * centered.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
    if (solver.info() != Eigen::Success ||
        solver.eigenvalues()(1) < 1e-10)
    {
        return false;
    }
    *normal = solver.eigenvectors().col(0).normalized();
    *offset = -normal->dot(centroid);
    for (const ShadowPoint &point : neighbors)
    {
        const double distance = std::abs(
            normal->dot(Eigen::Vector3d(point.x, point.y, point.z)) + *offset);
        if (distance > threshold)
        {
            return false;
        }
    }
    return true;
}

CorrespondenceSet correspondences(
    const LatestScanSnapshot &scan,
    const Pose3d &pose,
    const ShadowMapResult &map,
    const RegistrationConfig &config)
{
    CorrespondenceSet result;
    if (!scan.sonar_points || !map.tree)
    {
        return result;
    }
    result.jacobians.reserve(scan.sonar_points->size());
    result.residuals.reserve(scan.sonar_points->size());
    const Pose3d normalized_pose = pose.normalized();
    const Pose3d extrinsic = scan.T_vehicle_sonar.normalized();
    const double maximum_squared_distance =
        config.maximum_neighbor_distance_m * config.maximum_neighbor_distance_m;
    for (const PointXYZI &point : *scan.sonar_points)
    {
        const Eigen::Vector3d sonar(point.x, point.y, point.z);
        const Eigen::Vector3d vehicle =
            extrinsic.rotation * sonar + extrinsic.translation;
        const Eigen::Vector3d world =
            normalized_pose.rotation * vehicle + normalized_pose.translation;
        ShadowPoint query;
        query.x = static_cast<float>(world.x());
        query.y = static_cast<float>(world.y());
        query.z = static_cast<float>(world.z());
        ShadowPointVector nearest;
        std::vector<float> squared_distances;
        map.tree->Nearest_Search(query, kNearestNeighbors, nearest,
                                 squared_distances);
        if (nearest.size() < kNearestNeighbors || squared_distances.size() < kNearestNeighbors ||
            squared_distances.back() > maximum_squared_distance)
        {
            continue;
        }
        Eigen::Vector3d normal;
        double offset = 0.0;
        if (!fit_plane(nearest, config.plane_fit_threshold_m, &normal, &offset))
        {
            continue;
        }
        double residual = normal.dot(world) + offset;
        const double absolute_residual = std::abs(residual);
        const double weight = absolute_residual <= config.robust_huber_delta_m
                                  ? 1.0
                                  : config.robust_huber_delta_m / absolute_residual;
        const double square_root_weight = std::sqrt(weight);
        Eigen::Matrix<double, 1, 6> jacobian;
        const Eigen::Vector3d body_normal =
            normalized_pose.rotation.conjugate() * normal;
        jacobian << normal.transpose(),
            (vehicle.cross(body_normal)).transpose();
        result.jacobians.push_back(square_root_weight * jacobian);
        result.residuals.push_back(square_root_weight * residual);
    }
    return result;
}

std::pair<double, double> residual_statistics(
    const CorrespondenceSet &correspondences)
{
    if (correspondences.residuals.empty())
    {
        return {std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()};
    }
    std::vector<double> absolute;
    absolute.reserve(correspondences.residuals.size());
    double sum = 0.0;
    for (const double residual : correspondences.residuals)
    {
        const double value = std::abs(residual);
        absolute.push_back(value);
        sum += value;
    }
    const std::size_t p95_index = static_cast<std::size_t>(
        std::floor(0.95 * static_cast<double>(absolute.size() - 1)));
    std::nth_element(absolute.begin(), absolute.begin() + p95_index,
                     absolute.end());
    return {sum / static_cast<double>(absolute.size()), absolute[p95_index]};
}

}  // namespace

LatestScanRegistrar::LatestScanRegistrar(RegistrationConfig config)
    : config_(config)
{
    if (config_.maximum_iterations < 1 ||
        config_.minimum_effective_points < 6 ||
        config_.maximum_neighbor_distance_m <= 0.0 ||
        config_.plane_fit_threshold_m <= 0.0 ||
        config_.robust_huber_delta_m <= 0.0)
    {
        throw std::invalid_argument("Invalid latest-scan registration configuration");
    }
}

RegistrationResult LatestScanRegistrar::register_scan(
    const LatestScanSnapshot &scan,
    const PoseGraphSnapshot &graph,
    const ShadowMapResult &shadow_map) const
{
    const auto started = Clock::now();
    RegistrationResult result;
    result.graph_version = graph.version;
    result.shadow_tree_generation = shadow_map.shadow_tree_generation;
    result.scan_generation = scan.scan_generation;
    result.source_active_tree_generation = scan.active_tree_generation;
    result.scan_timestamp = scan.timestamp;
    result.T_local_vehicle_raw = scan.T_local_vehicle_raw;
    result.graph_anchor_covariance_position_rotation.block<3, 3>(0, 0) =
        graph.latest_optimized_covariance_graph.block<3, 3>(3, 3);
    result.graph_anchor_covariance_position_rotation.block<3, 3>(0, 3) =
        graph.latest_optimized_covariance_graph.block<3, 3>(3, 0);
    result.graph_anchor_covariance_position_rotation.block<3, 3>(3, 0) =
        graph.latest_optimized_covariance_graph.block<3, 3>(0, 3);
    result.graph_anchor_covariance_position_rotation.block<3, 3>(3, 3) =
        graph.latest_optimized_covariance_graph.block<3, 3>(0, 0);
    const auto reject = [&](const std::string &reason) {
        result.reason = reason;
        result.registration_time_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - started).count();
        return result;
    };

    if (!scan.T_local_vehicle_raw.finite() || !scan.T_vehicle_sonar.finite() ||
        !scan.sonar_points || scan.sonar_points->empty() || !shadow_map.valid ||
        !shadow_map.tree || graph.raw_poses.empty() ||
        graph.raw_poses.size() != graph.optimized_poses.size())
    {
        return reject("invalid_registration_input");
    }
    if (shadow_map.graph_version != graph.version)
    {
        return reject("stale_shadow_graph_version");
    }

    const Pose3d delayed_correction = compose(
        graph.optimized_poses.back(), inverse(graph.raw_poses.back()));
    Pose3d estimate = compose(delayed_correction, scan.T_local_vehicle_raw);
    result.T_local_vehicle_initial = estimate;
    CorrespondenceSet initial = correspondences(scan, estimate, shadow_map, config_);
    result.initial_effective_points = initial.residuals.size();
    std::tie(result.initial_residual_mean_m, result.initial_residual_p95_m) =
        residual_statistics(initial);
    if (initial.residuals.size() < config_.minimum_effective_points)
    {
        return reject("too_few_initial_correspondences");
    }

    Matrix6d final_information = Matrix6d::Zero();
    double final_squared_error = 0.0;
    for (int iteration = 0; iteration < config_.maximum_iterations; ++iteration)
    {
        const CorrespondenceSet matches =
            iteration == 0 ? initial : correspondences(scan, estimate, shadow_map, config_);
        if (matches.residuals.size() < config_.minimum_effective_points)
        {
            return reject("too_few_correspondences_during_iteration");
        }
        Matrix6d information = Matrix6d::Zero();
        Eigen::Matrix<double, 6, 1> gradient =
            Eigen::Matrix<double, 6, 1>::Zero();
        double squared_error = 0.0;
        for (std::size_t index = 0; index < matches.residuals.size(); ++index)
        {
            information.noalias() += matches.jacobians[index].transpose() *
                                     matches.jacobians[index];
            gradient.noalias() += matches.jacobians[index].transpose() *
                                  matches.residuals[index];
            squared_error += matches.residuals[index] * matches.residuals[index];
        }
        information = 0.5 * (information + information.transpose());
        Eigen::SelfAdjointEigenSolver<Matrix6d> eigen_solver(information);
        if (eigen_solver.info() != Eigen::Success)
        {
            return reject("information_eigendecomposition_failed");
        }
        const double minimum = eigen_solver.eigenvalues().minCoeff();
        const double maximum = eigen_solver.eigenvalues().maxCoeff();
        result.information_min_eigenvalue = minimum;
        result.information_condition =
            minimum > 0.0 ? maximum / minimum
                          : std::numeric_limits<double>::infinity();
        if (minimum < config_.minimum_information_eigenvalue ||
            result.information_condition > config_.maximum_information_condition)
        {
            return reject("underconstrained_full_se3_geometry");
        }
        Eigen::LDLT<Matrix6d> decomposition(information);
        if (decomposition.info() != Eigen::Success)
        {
            return reject("normal_equation_factorization_failed");
        }
        const Eigen::Matrix<double, 6, 1> increment =
            decomposition.solve(-gradient);
        if (!increment.allFinite())
        {
            return reject("non_finite_registration_increment");
        }
        final_information = information;
        final_squared_error = squared_error;
        result.iterations = iteration + 1;

        Pose3d accepted_pose = estimate;
        double accepted_scale = 0.0;
        double best_mean_squared_error =
            squared_error / static_cast<double>(matches.residuals.size());
        for (const double scale : {1.0, 0.5, 0.25, 0.125})
        {
            Pose3d candidate = estimate;
            candidate.translation += scale * increment.head<3>();
            candidate.rotation =
                (candidate.rotation * exp_so3(scale * increment.tail<3>()))
                    .normalized();
            const CorrespondenceSet candidate_matches =
                correspondences(scan, candidate, shadow_map, config_);
            if (candidate_matches.residuals.size() <
                config_.minimum_effective_points)
            {
                continue;
            }
            double candidate_squared_error = 0.0;
            for (const double residual : candidate_matches.residuals)
            {
                candidate_squared_error += residual * residual;
            }
            const double candidate_mean_squared_error =
                candidate_squared_error /
                static_cast<double>(candidate_matches.residuals.size());
            if (candidate_mean_squared_error + 1e-12 < best_mean_squared_error)
            {
                accepted_pose = candidate;
                accepted_scale = scale;
                break;
            }
        }
        if (accepted_scale == 0.0)
        {
            // The graph-propagated pose is already a local point-to-plane minimum.
            result.converged = true;
            break;
        }
        estimate = accepted_pose;
        if (accepted_scale * increment.head<3>().norm() <=
                config_.convergence_translation_m &&
            accepted_scale * increment.tail<3>().norm() <=
                config_.convergence_rotation_rad)
        {
            result.converged = true;
            break;
        }
    }
    if (!result.converged)
    {
        return reject("maximum_iterations_reached");
    }

    const CorrespondenceSet final_matches =
        correspondences(scan, estimate, shadow_map, config_);
    result.final_effective_points = final_matches.residuals.size();
    std::tie(result.final_residual_mean_m, result.final_residual_p95_m) =
        residual_statistics(final_matches);
    if (result.final_effective_points < config_.minimum_effective_points)
    {
        return reject("too_few_final_correspondences");
    }
    final_information.setZero();
    final_squared_error = 0.0;
    for (std::size_t index = 0; index < final_matches.residuals.size(); ++index)
    {
        final_information.noalias() +=
            final_matches.jacobians[index].transpose() *
            final_matches.jacobians[index];
        final_squared_error += final_matches.residuals[index] *
                               final_matches.residuals[index];
    }
    final_information =
        0.5 * (final_information + final_information.transpose());
    Eigen::SelfAdjointEigenSolver<Matrix6d> final_eigen_solver(final_information);
    if (final_eigen_solver.info() != Eigen::Success)
    {
        return reject("final_information_eigendecomposition_failed");
    }
    result.information_min_eigenvalue =
        final_eigen_solver.eigenvalues().minCoeff();
    result.information_condition =
        result.information_min_eigenvalue > 0.0
            ? final_eigen_solver.eigenvalues().maxCoeff() /
                  result.information_min_eigenvalue
            : std::numeric_limits<double>::infinity();
    if (result.information_min_eigenvalue <
            config_.minimum_information_eigenvalue ||
        result.information_condition > config_.maximum_information_condition)
    {
        return reject("underconstrained_final_full_se3_geometry");
    }
    const double residual_tolerance = 1e-8;
    if (result.final_residual_mean_m >
        result.initial_residual_mean_m + residual_tolerance)
    {
        return reject("registration_residual_did_not_improve");
    }
    const Pose3d registration_delta = between(
        result.T_local_vehicle_initial, estimate);
    if (registration_delta.translation.norm() >
            config_.maximum_registration_translation_m ||
        registration_delta.rotation.angularDistance(Eigen::Quaterniond::Identity()) >
            config_.maximum_registration_rotation_rad)
    {
        return reject("registration_correction_too_large");
    }

    const double degrees_of_freedom = std::max(
        1.0, static_cast<double>(result.final_effective_points) - 6.0);
    double sigma_squared = final_squared_error / degrees_of_freedom;
    sigma_squared = std::max(sigma_squared, 1e-10);
    Eigen::LDLT<Matrix6d> final_decomposition(final_information);
    if (final_decomposition.info() != Eigen::Success)
    {
        return reject("final_normal_equation_factorization_failed");
    }
    result.covariance = sigma_squared *
                        final_decomposition.solve(Matrix6d::Identity());
    result.covariance = 0.5 *
                        (result.covariance + result.covariance.transpose());
    if (!covariance_is_valid(result.covariance, 1e-9))
    {
        return reject("invalid_registration_covariance");
    }
    result.T_local_vehicle_registered = estimate;
    result.valid = true;
    result.reason = "ready";
    result.registration_time_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - started).count();
    return result;
}

}  // namespace uwfl2::loop_closure

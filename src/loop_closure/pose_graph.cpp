#include "loop_closure/pose_graph.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace uwfl2::loop_closure
{
namespace
{

gtsam::Key pose_key(std::uint64_t id)
{
    return gtsam::Symbol('x', id);
}

Matrix6d force_positive_definite(Matrix6d covariance, const PoseGraphConfig &config)
{
    covariance = 0.5 * (covariance + covariance.transpose());
    for (int axis = 0; axis < 3; ++axis)
    {
        covariance(axis, axis) =
            std::max(covariance(axis, axis), config.odometry_rotation_variance_floor);
        covariance(axis + 3, axis + 3) =
            std::max(covariance(axis + 3, axis + 3),
                     config.odometry_translation_variance_floor);
    }
    Eigen::SelfAdjointEigenSolver<Matrix6d> solver(covariance);
    if (solver.info() != Eigen::Success)
    {
        throw std::runtime_error("Pose covariance eigendecomposition failed");
    }
    Eigen::Matrix<double, 6, 1> eigenvalues = solver.eigenvalues();
    for (int index = 0; index < eigenvalues.size(); ++index)
    {
        eigenvalues(index) = std::max(eigenvalues(index), 1e-12);
    }
    return solver.eigenvectors() * eigenvalues.asDiagonal() *
           solver.eigenvectors().transpose();
}

gtsam::ISAM2Params isam_parameters()
{
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    return parameters;
}

struct ResidualMagnitude
{
    double translation = 0.0;
    double rotation = 0.0;
};

ResidualMagnitude relative_pose_residual(
    const gtsam::Pose3 &from,
    const gtsam::Pose3 &to,
    const gtsam::Pose3 &measurement)
{
    const gtsam::Pose3 predicted = from.between(to);
    const gtsam::Pose3 error = measurement.between(predicted);
    return {error.translation().norm(), error.rotation().axisAngle().second};
}

bool finite_pose(const gtsam::Pose3 &pose)
{
    return pose.matrix().allFinite();
}

}  // namespace

FullSe3PoseGraph::FullSe3PoseGraph(PoseGraphConfig config) : config_(config)
{
    isam_ = gtsam::ISAM2(isam_parameters());
}

gtsam::Pose3 FullSe3PoseGraph::to_gtsam(const Pose3d &pose)
{
    const Pose3d normalized = pose.normalized();
    const auto &q = normalized.rotation;
    return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()),
                        gtsam::Point3(normalized.translation));
}

Pose3d FullSe3PoseGraph::from_gtsam(const gtsam::Pose3 &pose)
{
    const Eigen::Matrix3d rotation_matrix = pose.rotation().matrix();
    return {Eigen::Quaterniond(rotation_matrix).normalized(), pose.translation()};
}

Matrix6d FullSe3PoseGraph::sanitized_odometry_covariance(
    const Matrix6d &previous,
    const Matrix6d &current,
    const PoseGraphConfig &config)
{
    // This conservative first-order approximation intentionally does not claim
    // independence-aware cross-time covariance that the front end does not retain.
    return force_positive_definite(previous + current, config);
}

std::uint64_t FullSe3PoseGraph::append_keyframe(const Keyframe &keyframe)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if ((!keyframes_.empty() && keyframe.id <= keyframes_.back().id) ||
        (keyframes_.empty() && keyframe.id != 0))
    {
        throw std::invalid_argument("Keyframe IDs must be unique and monotonically increasing");
    }
    if (!keyframe.T_local_vehicle.finite() ||
        !covariance_is_valid(keyframe.pose_covariance))
    {
        throw std::invalid_argument("Keyframe pose or covariance is invalid");
    }

    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;
    const gtsam::Key current_key = pose_key(keyframe.id);
    const gtsam::Pose3 current_pose = to_gtsam(keyframe.T_local_vehicle);
    new_values.insert(current_key, current_pose);

    if (keyframes_.empty())
    {
        gtsam::Vector6 sigmas;
        sigmas << config_.prior_rotation_sigma_rad,
            config_.prior_rotation_sigma_rad,
            config_.prior_rotation_sigma_rad,
            config_.prior_translation_sigma_m,
            config_.prior_translation_sigma_m,
            config_.prior_translation_sigma_m;
        new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
            current_key, current_pose, gtsam::noiseModel::Diagonal::Sigmas(sigmas)));
    }
    else
    {
        const Keyframe &previous = keyframes_.back();
        const Pose3d relative = between(previous.T_local_vehicle,
                                        keyframe.T_local_vehicle);
        const Matrix6d covariance = sanitized_odometry_covariance(
            previous.pose_covariance, keyframe.pose_covariance, config_);
        new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
            pose_key(previous.id), current_key, to_gtsam(relative),
            gtsam::noiseModel::Gaussian::Covariance(covariance)));
    }

    isam_.update(new_factors, new_values);
    isam_.update();
    graph_.push_back(new_factors);
    estimate_ = isam_.calculateEstimate();
    keyframes_.push_back(keyframe);
    ++version_;
    return version_;
}

LoopEvaluation FullSe3PoseGraph::try_add_loop(const LoopConstraint &constraint)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto started = std::chrono::steady_clock::now();
    LoopEvaluation result;
    result.graph_version = version_;

    const auto reject = [&](const std::string &reason) {
        result.accepted = false;
        result.reason = reason;
        result.optimization_time_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started)
                .count();
        return result;
    };

    if (constraint.from_id == constraint.to_id)
    {
        return reject("same_keyframe");
    }
    if (!constraint.T_from_to.finite() ||
        !covariance_is_valid(constraint.covariance))
    {
        return reject("non_finite_or_invalid_covariance");
    }

    const auto from_iterator = std::find_if(
        keyframes_.begin(), keyframes_.end(), [&](const Keyframe &keyframe) {
            return keyframe.id == constraint.from_id;
        });
    const auto to_iterator = std::find_if(
        keyframes_.begin(), keyframes_.end(), [&](const Keyframe &keyframe) {
            return keyframe.id == constraint.to_id;
        });
    if (from_iterator == keyframes_.end() || to_iterator == keyframes_.end())
    {
        return reject("unknown_keyframe");
    }
    const std::size_t from_index =
        static_cast<std::size_t>(std::distance(keyframes_.begin(), from_iterator));
    const std::size_t to_index =
        static_cast<std::size_t>(std::distance(keyframes_.begin(), to_iterator));
    if (from_index >= to_index)
    {
        return reject("keyframes_not_chronological");
    }
    if (!constraint.test_override &&
        to_index - from_index < config_.loop_minimum_keyframe_separation)
    {
        return reject("insufficient_keyframe_separation");
    }

    const gtsam::Key from_key = pose_key(constraint.from_id);
    const gtsam::Key to_key = pose_key(constraint.to_id);
    const gtsam::Pose3 measurement = to_gtsam(constraint.T_from_to);
    const ResidualMagnitude initial_residual = relative_pose_residual(
        estimate_.at<gtsam::Pose3>(from_key),
        estimate_.at<gtsam::Pose3>(to_key), measurement);
    result.loop_translation_error_before = initial_residual.translation;
    result.loop_rotation_error_before_rad = initial_residual.rotation;
    if (!constraint.test_override &&
        (initial_residual.translation >
             config_.loop_maximum_initial_translation_error_m ||
         initial_residual.rotation > config_.loop_maximum_initial_rotation_error_rad))
    {
        return reject("initial_loop_residual_too_large");
    }

    try
    {
        gtsam::NonlinearFactorGraph candidate_graph = graph_;
        const Matrix6d covariance = force_positive_definite(
            constraint.covariance, config_);
        candidate_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            from_key, to_key, measurement,
            gtsam::noiseModel::Gaussian::Covariance(covariance)));
        result.graph_error_before = candidate_graph.error(estimate_);

        gtsam::LevenbergMarquardtParams parameters;
        parameters.setVerbosityLM("SILENT");
        parameters.setMaxIterations(50);
        const gtsam::Values candidate_estimate =
            gtsam::LevenbergMarquardtOptimizer(
                candidate_graph, estimate_, parameters)
                .optimize();
        result.graph_error_after = candidate_graph.error(candidate_estimate);

        if (!std::isfinite(result.graph_error_before) ||
            !std::isfinite(result.graph_error_after) ||
            result.graph_error_after > result.graph_error_before +
                                           std::max(1e-9, 1e-9 * result.graph_error_before))
        {
            return reject("optimization_did_not_reduce_error");
        }

        const ResidualMagnitude final_residual = relative_pose_residual(
            candidate_estimate.at<gtsam::Pose3>(from_key),
            candidate_estimate.at<gtsam::Pose3>(to_key), measurement);
        result.loop_translation_error_after = final_residual.translation;
        result.loop_rotation_error_after_rad = final_residual.rotation;
        if (final_residual.translation > initial_residual.translation + 1e-9 ||
            final_residual.rotation > initial_residual.rotation + 1e-9)
        {
            return reject("loop_residual_increased");
        }

        for (const Keyframe &keyframe : keyframes_)
        {
            const gtsam::Key key = pose_key(keyframe.id);
            const gtsam::Pose3 before = estimate_.at<gtsam::Pose3>(key);
            const gtsam::Pose3 after = candidate_estimate.at<gtsam::Pose3>(key);
            if (!finite_pose(after))
            {
                return reject("non_finite_optimized_pose");
            }
            const ResidualMagnitude correction = relative_pose_residual(
                before, after, gtsam::Pose3());
            if (correction.translation >
                    config_.loop_maximum_pose_correction_translation_m ||
                correction.rotation >
                    config_.loop_maximum_pose_correction_rotation_rad)
            {
                return reject("optimized_pose_correction_too_large");
            }
        }

        gtsam::ISAM2 replacement(isam_parameters());
        replacement.update(candidate_graph, candidate_estimate);
        replacement.update();
        gtsam::Values replacement_estimate = replacement.calculateEstimate();
        for (const Keyframe &keyframe : keyframes_)
        {
            if (!finite_pose(
                    replacement_estimate.at<gtsam::Pose3>(pose_key(keyframe.id))))
            {
                return reject("non_finite_committed_pose");
            }
        }

        graph_ = std::move(candidate_graph);
        estimate_ = std::move(replacement_estimate);
        isam_ = std::move(replacement);
        ++loop_factor_count_;
        ++version_;
        result.accepted = true;
        result.reason = "accepted";
        result.graph_version = version_;
        result.optimization_time_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started)
                .count();
        return result;
    }
    catch (const std::exception &exception)
    {
        return reject(std::string("optimization_exception: ") + exception.what());
    }
}

PoseGraphSnapshot FullSe3PoseGraph::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    PoseGraphSnapshot result;
    result.version = version_;
    result.node_count = keyframes_.size();
    result.factor_count = graph_.size();
    result.loop_factor_count = loop_factor_count_;
    result.ids.reserve(keyframes_.size());
    result.timestamps.reserve(keyframes_.size());
    result.raw_poses.reserve(keyframes_.size());
    result.optimized_poses.reserve(keyframes_.size());
    result.keyframes.reserve(keyframes_.size());
    for (const Keyframe &keyframe : keyframes_)
    {
        result.ids.push_back(keyframe.id);
        result.timestamps.push_back(keyframe.timestamp);
        result.keyframes.push_back(keyframe);
        result.raw_poses.push_back(keyframe.T_local_vehicle);
        result.optimized_poses.push_back(
            from_gtsam(estimate_.at<gtsam::Pose3>(pose_key(keyframe.id))));
    }
    if (!keyframes_.empty())
    {
        result.latest_optimized_covariance_graph =
            isam_.marginalCovariance(pose_key(keyframes_.back().id));
    }
    return result;
}

}  // namespace uwfl2::loop_closure

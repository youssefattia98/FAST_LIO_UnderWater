#include "loop_closure/pose_graph.hpp"

#include <algorithm>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
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

}  // namespace

FullSe3PoseGraph::FullSe3PoseGraph(PoseGraphConfig config) : config_(config)
{
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam_ = gtsam::ISAM2(parameters);
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

PoseGraphSnapshot FullSe3PoseGraph::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    PoseGraphSnapshot result;
    result.version = version_;
    result.node_count = keyframes_.size();
    result.factor_count = graph_.size();
    result.raw_poses.reserve(keyframes_.size());
    result.optimized_poses.reserve(keyframes_.size());
    for (const Keyframe &keyframe : keyframes_)
    {
        result.raw_poses.push_back(keyframe.T_local_vehicle);
        result.optimized_poses.push_back(
            from_gtsam(estimate_.at<gtsam::Pose3>(pose_key(keyframe.id))));
    }
    return result;
}

}  // namespace uwfl2::loop_closure

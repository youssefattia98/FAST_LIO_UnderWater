#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace uwfl2::loop_closure
{

using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct Pose3d
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    bool finite() const
    {
        return rotation.coeffs().allFinite() && translation.allFinite() &&
               rotation.norm() > std::numeric_limits<double>::epsilon();
    }

    Pose3d normalized() const
    {
        Pose3d result = *this;
        result.rotation.normalize();
        return result;
    }
};

inline Pose3d compose(const Pose3d &lhs, const Pose3d &rhs)
{
    const Pose3d a = lhs.normalized();
    const Pose3d b = rhs.normalized();
    return {(a.rotation * b.rotation).normalized(),
            a.translation + a.rotation * b.translation};
}

inline Pose3d inverse(const Pose3d &pose)
{
    const Eigen::Quaterniond inverse_rotation = pose.normalized().rotation.conjugate();
    return {inverse_rotation, -(inverse_rotation * pose.translation)};
}

// Returns T_a_b = inverse(T_l_a) * T_l_b.
inline Pose3d between(const Pose3d &T_l_a, const Pose3d &T_l_b)
{
    return compose(inverse(T_l_a), T_l_b);
}

inline double rotation_distance_rad(const Pose3d &a, const Pose3d &b)
{
    return a.normalized().rotation.angularDistance(b.normalized().rotation);
}

inline bool covariance_is_valid(const Matrix6d &covariance, double tolerance = 1e-10)
{
    if (!covariance.allFinite())
    {
        return false;
    }
    const Matrix6d symmetric = 0.5 * (covariance + covariance.transpose());
    Eigen::SelfAdjointEigenSolver<Matrix6d> solver(symmetric, Eigen::EigenvaluesOnly);
    return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -tolerance;
}

// IKFOM orders the pose error as [position, right-attitude]. The graph uses
// GTSAM Pose3 tangent ordering [rotation, translation].
template <typename Covariance>
Matrix6d extract_graph_pose_covariance(const Covariance &ikf_covariance)
{
    Matrix6d graph_covariance = Matrix6d::Zero();
    graph_covariance.template block<3, 3>(0, 0) =
        ikf_covariance.template block<3, 3>(3, 3);
    graph_covariance.template block<3, 3>(0, 3) =
        ikf_covariance.template block<3, 3>(3, 0);
    graph_covariance.template block<3, 3>(3, 0) =
        ikf_covariance.template block<3, 3>(0, 3);
    graph_covariance.template block<3, 3>(3, 3) =
        ikf_covariance.template block<3, 3>(0, 0);
    return 0.5 * (graph_covariance + graph_covariance.transpose());
}

struct PointXYZI
{
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
};

struct Keyframe
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::uint64_t id = 0;
    double timestamp = 0.0;
    Pose3d T_local_vehicle;
    Matrix6d pose_covariance = Matrix6d::Identity();
    Pose3d T_vehicle_sonar;
    std::shared_ptr<const std::vector<PointXYZI>> sonar_points;
    // Compact scan history already expressed in the raw camera_init frame.
    // Keeping it attached to its pose makes global map correction reversible.
    std::shared_ptr<const std::vector<PointXYZI>> map_points_world;
    std::uint64_t graph_version = 0;
    std::uint64_t tree_generation = 0;
};

struct LoopConstraint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::uint64_t from_id = 0;
    std::uint64_t to_id = 0;
    Pose3d T_from_to;
    Matrix6d covariance = Matrix6d::Identity();
    bool test_override = false;
};

struct LoopEvaluation
{
    bool accepted = false;
    std::string reason;
    std::uint64_t graph_version = 0;
    double graph_error_before = 0.0;
    double graph_error_after = 0.0;
    double loop_translation_error_before = 0.0;
    double loop_translation_error_after = 0.0;
    double loop_rotation_error_before_rad = 0.0;
    double loop_rotation_error_after_rad = 0.0;
    double initial_nis = 0.0;
    double optimization_time_ms = 0.0;
};

struct KeyframeSelectionConfig
{
    double translation_m = 1.0;
    double rotation_rad = 10.0 * 3.14159265358979323846 / 180.0;
    double minimum_interval_s = 0.5;
    double maximum_interval_s = 5.0;
    std::size_t minimum_points = 20;
};

class KeyframeSelector
{
public:
    explicit KeyframeSelector(KeyframeSelectionConfig config) : config_(config) {}

    bool should_select(double timestamp, const Pose3d &pose, std::size_t point_count) const
    {
        if (!std::isfinite(timestamp) || !pose.finite() ||
            point_count < config_.minimum_points)
        {
            return false;
        }
        if (!has_last_)
        {
            return true;
        }
        const double elapsed = timestamp - last_timestamp_;
        if (elapsed < config_.minimum_interval_s)
        {
            return false;
        }
        const double translation = (pose.translation - last_pose_.translation).norm();
        const double rotation = rotation_distance_rad(last_pose_, pose);
        return translation >= config_.translation_m || rotation >= config_.rotation_rad ||
               elapsed >= config_.maximum_interval_s;
    }

    void accept(double timestamp, const Pose3d &pose)
    {
        has_last_ = true;
        last_timestamp_ = timestamp;
        last_pose_ = pose.normalized();
    }

private:
    KeyframeSelectionConfig config_;
    bool has_last_ = false;
    double last_timestamp_ = 0.0;
    Pose3d last_pose_;
};

}  // namespace uwfl2::loop_closure

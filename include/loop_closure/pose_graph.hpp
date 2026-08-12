#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>

#include "loop_closure/loop_closure_types.hpp"

namespace uwfl2::loop_closure
{

struct PoseGraphConfig
{
    double prior_rotation_sigma_rad = 1e-4;
    double prior_translation_sigma_m = 1e-4;
    double odometry_rotation_variance_floor = 1e-8;
    double odometry_translation_variance_floor = 1e-6;
};

struct PoseGraphSnapshot
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::uint64_t version = 0;
    std::size_t node_count = 0;
    std::size_t factor_count = 0;
    std::vector<Pose3d, Eigen::aligned_allocator<Pose3d>> raw_poses;
    std::vector<Pose3d, Eigen::aligned_allocator<Pose3d>> optimized_poses;
};

class FullSe3PoseGraph
{
public:
    explicit FullSe3PoseGraph(PoseGraphConfig config = {});

    std::uint64_t append_keyframe(const Keyframe &keyframe);
    PoseGraphSnapshot snapshot() const;

    static gtsam::Pose3 to_gtsam(const Pose3d &pose);
    static Pose3d from_gtsam(const gtsam::Pose3 &pose);

private:
    static Matrix6d sanitized_odometry_covariance(
        const Matrix6d &previous,
        const Matrix6d &current,
        const PoseGraphConfig &config);

    PoseGraphConfig config_;
    mutable std::mutex mutex_;
    gtsam::ISAM2 isam_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values estimate_;
    std::vector<Keyframe, Eigen::aligned_allocator<Keyframe>> keyframes_;
    std::uint64_t version_ = 0;
};

}  // namespace uwfl2::loop_closure

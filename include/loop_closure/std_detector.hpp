#pragma once

// ROS-independent implementation inspired by LTA-OM/STD (Apache-2.0):
// voxel-plane keypoints, binary signatures, triangle hashing, candidate voting,
// rigid full-SE(3) RANSAC, and 3D geometric verification. No ROS 1, Ceres, or
// patched-GTSAM code is carried over.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "loop_closure/loop_closure_types.hpp"

namespace uwfl2::loop_closure
{

struct StdConfig
{
    std::size_t minimum_keyframe_separation = 20;
    double voxel_size_m = 0.6;
    std::size_t minimum_voxel_points = 5;
    double plane_eigenvalue_ratio = 0.08;
    std::size_t maximum_keypoints = 80;
    std::size_t triangle_neighbor_count = 8;
    std::size_t maximum_triangles = 1000;
    double triangle_minimum_side_m = 0.4;
    double triangle_maximum_side_m = 12.0;
    double triangle_side_resolution_m = 0.20;
    double triangle_relative_error = 0.08;
    double binary_similarity_minimum = 0.25;
    std::size_t minimum_triangle_matches = 5;
    std::size_t maximum_candidates = 8;
    std::size_t minimum_ransac_inliers = 5;
    double ransac_vertex_threshold_m = 0.35;
    double geometric_overlap_distance_m = 0.35;
    double geometric_overlap_minimum = 0.20;
    std::size_t refinement_iterations = 6;
    std::size_t required_consistent_detections = 2;
    double single_detection_overlap_minimum = 0.80;
    double confirmation_translation_m = 1.0;
    double confirmation_rotation_rad = 10.0 * 3.14159265358979323846 / 180.0;
    std::size_t confirmation_target_id_tolerance = 5;
    double maximum_pose_distance_m = 15.0;
    double maximum_prior_translation_error_m = 10.0;
    double maximum_prior_rotation_rad = 45.0 * 3.14159265358979323846 / 180.0;
};

struct StdDetectionResult
{
    bool proposed = false;
    bool confirmed = false;
    std::string reason;
    std::uint64_t source_id = 0;
    std::uint64_t target_id = 0;
    std::size_t keypoints = 0;
    std::size_t triangles = 0;
    std::size_t descriptor_matches = 0;
    std::size_t ransac_inliers = 0;
    double overlap = 0.0;
    double descriptor_time_ms = 0.0;
    double search_time_ms = 0.0;
    double verification_time_ms = 0.0;
    std::optional<LoopConstraint> loop;
};

class StableTriangleDetector
{
public:
    explicit StableTriangleDetector(StdConfig config = {});
    ~StableTriangleDetector();

    StdDetectionResult process(const Keyframe &keyframe);
    std::size_t database_size() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace uwfl2::loop_closure

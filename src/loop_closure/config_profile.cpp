#include "loop_closure/config_profile.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace uwfl2::loop_closure
{
namespace
{

constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;

template <typename T>
T required(const YAML::Node &node, const char *key)
{
    if (!node || !node[key])
    {
        throw std::invalid_argument(std::string("Missing loop-closure profile key: ") + key);
    }
    return node[key].as<T>();
}

template <typename T>
T optional(const YAML::Node &node, const char *key, const T &fallback)
{
    return node && node[key] ? node[key].as<T>() : fallback;
}

std::size_t required_size(const YAML::Node &node, const char *key)
{
    const int value = required<int>(node, key);
    if (value < 1)
    {
        throw std::invalid_argument(std::string("Loop-closure profile key must be positive: ") + key);
    }
    return static_cast<std::size_t>(value);
}

}  // namespace

LoopClosureConfig load_config_profile(const std::filesystem::path &path)
{
    const YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node config = root["loop_closure"];
    if (!config)
    {
        throw std::invalid_argument("Loop-closure profile has no 'loop_closure' root: " + path.string());
    }

    LoopClosureConfig result;
    result.automatic_detection_enabled = true;
    result.queue_capacity = required_size(config, "queue_capacity");

    const YAML::Node keyframes = config["keyframes"];
    result.keyframes.translation_m = required<double>(keyframes, "translation_m");
    result.keyframes.rotation_rad =
        required<double>(keyframes, "rotation_deg") * kRadiansPerDegree;
    result.keyframes.minimum_interval_s =
        required<double>(keyframes, "minimum_interval_s");
    result.keyframes.maximum_interval_s =
        required<double>(keyframes, "maximum_interval_s");
    result.keyframes.minimum_points = required_size(keyframes, "minimum_points");

    const YAML::Node graph = config["pose_graph"];
    result.pose_graph.prior_rotation_sigma_rad =
        required<double>(graph, "prior_rotation_sigma_rad");
    result.pose_graph.prior_translation_sigma_m =
        required<double>(graph, "prior_translation_sigma_m");
    result.pose_graph.odometry_rotation_variance_floor =
        required<double>(graph, "odometry_rotation_variance_floor");
    result.pose_graph.odometry_translation_variance_floor =
        required<double>(graph, "odometry_translation_variance_floor");
    result.pose_graph.loop_minimum_keyframe_separation =
        required_size(graph, "loop_minimum_keyframe_separation");
    result.pose_graph.loop_maximum_initial_translation_error_m =
        required<double>(graph, "loop_maximum_initial_translation_error_m");
    result.pose_graph.loop_minimum_initial_rotation_error_rad =
        required<double>(graph, "loop_minimum_initial_rotation_error_deg") *
        kRadiansPerDegree;
    result.pose_graph.loop_maximum_initial_rotation_error_rad =
        required<double>(graph, "loop_maximum_initial_rotation_error_deg") *
        kRadiansPerDegree;
    result.pose_graph.loop_minimum_initial_nis =
        required<double>(graph, "loop_minimum_initial_nis");
    result.pose_graph.loop_maximum_initial_nis =
        required<double>(graph, "loop_maximum_initial_nis");
    result.pose_graph.loop_maximum_pose_correction_translation_m =
        required<double>(graph, "loop_maximum_pose_correction_translation_m");
    result.pose_graph.loop_maximum_pose_correction_rotation_rad =
        required<double>(graph, "loop_maximum_pose_correction_rotation_deg") *
        kRadiansPerDegree;
    result.pose_graph.loop_maximum_pose_correction_tilt_rad =
        optional<double>(graph, "loop_maximum_pose_correction_tilt_deg", 3.0) *
        kRadiansPerDegree;
    result.pose_graph.loop_maximum_rebase_translation_m =
        required<double>(graph, "loop_maximum_rebase_translation_m");
    result.pose_graph.loop_maximum_rebase_rotation_rad =
        required<double>(graph, "loop_maximum_rebase_rotation_deg") *
        kRadiansPerDegree;

    const YAML::Node map = config["shadow_map"];
    result.shadow_map.radius_m = required<double>(map, "radius_m");
    result.shadow_map.maximum_keyframes = required_size(map, "maximum_keyframes");
    result.shadow_map.maximum_input_points =
        required_size(map, "maximum_input_points");

    const YAML::Node registration = config["registration"];
    result.registration.maximum_iterations =
        required<int>(registration, "maximum_iterations");
    result.registration.maximum_refresh_attempts =
        required_size(registration, "maximum_refresh_attempts");
    result.registration.minimum_effective_points =
        required_size(registration, "minimum_effective_points");
    result.registration.maximum_neighbor_distance_m =
        required<double>(registration, "maximum_neighbor_distance_m");
    result.registration.plane_fit_threshold_m =
        required<double>(registration, "plane_fit_threshold_m");
    result.registration.robust_huber_delta_m =
        required<double>(registration, "robust_huber_delta_m");
    result.registration.convergence_translation_m =
        required<double>(registration, "convergence_translation_m");
    result.registration.convergence_rotation_rad =
        required<double>(registration, "convergence_rotation_deg") * kRadiansPerDegree;
    result.registration.minimum_information_eigenvalue =
        required<double>(registration, "minimum_information_eigenvalue");
    result.registration.maximum_information_condition =
        required<double>(registration, "maximum_information_condition");
    result.registration.maximum_registration_translation_m =
        required<double>(registration, "maximum_translation_m");
    result.registration.maximum_registration_rotation_rad =
        required<double>(registration, "maximum_rotation_deg") * kRadiansPerDegree;
    result.registration.protected_tilt_sigma_rad =
        optional<double>(registration, "protected_tilt_sigma_deg", 1.0) *
        kRadiansPerDegree;
    result.registration.protected_vertical_sigma_m =
        optional<double>(registration, "protected_vertical_sigma_m", 0.25);
    result.registration.maximum_registration_tilt_rad =
        optional<double>(registration, "maximum_tilt_deg", 3.0) *
        kRadiansPerDegree;

    const YAML::Node detector = config["std_detector"];
    result.std_detection.minimum_keyframe_separation =
        required_size(detector, "minimum_keyframe_separation");
    result.std_detection.minimum_loop_duration_s = 0.0;
    result.std_detection.voxel_size_m = required<double>(detector, "voxel_size_m");
    result.std_detection.minimum_voxel_points =
        required_size(detector, "minimum_voxel_points");
    result.std_detection.plane_eigenvalue_ratio =
        required<double>(detector, "plane_eigenvalue_ratio");
    result.std_detection.maximum_keypoints = required_size(detector, "maximum_keypoints");
    result.std_detection.triangle_neighbor_count =
        required_size(detector, "triangle_neighbor_count");
    result.std_detection.maximum_triangles = required_size(detector, "maximum_triangles");
    result.std_detection.maximum_references_per_bucket =
        required_size(detector, "maximum_references_per_bucket");
    result.std_detection.triangle_minimum_side_m =
        required<double>(detector, "triangle_minimum_side_m");
    result.std_detection.triangle_maximum_side_m =
        required<double>(detector, "triangle_maximum_side_m");
    result.std_detection.triangle_side_resolution_m =
        required<double>(detector, "triangle_side_resolution_m");
    result.std_detection.triangle_relative_error =
        required<double>(detector, "triangle_relative_error");
    result.std_detection.binary_similarity_minimum =
        required<double>(detector, "binary_similarity_minimum");
    result.std_detection.minimum_triangle_matches =
        required_size(detector, "minimum_triangle_matches");
    result.std_detection.maximum_candidates = required_size(detector, "maximum_candidates");
    result.std_detection.maximum_matches_per_candidate =
        required_size(detector, "maximum_matches_per_candidate");
    result.std_detection.maximum_ransac_hypotheses =
        required_size(detector, "maximum_ransac_hypotheses");
    result.std_detection.minimum_ransac_inliers =
        required_size(detector, "minimum_ransac_inliers");
    result.std_detection.ransac_vertex_threshold_m =
        required<double>(detector, "ransac_vertex_threshold_m");
    result.std_detection.geometric_overlap_distance_m =
        required<double>(detector, "geometric_overlap_distance_m");
    result.std_detection.geometric_overlap_minimum =
        required<double>(detector, "geometric_overlap_minimum");
    // Keep the legacy scalar keys as fallbacks so older profiles and tuner
    // outputs remain loadable. New profiles can independently express which
    // directions the sonar loop actually observes.
    const double legacy_rotation_sigma_deg =
        required<double>(detector, "loop_rotation_sigma_deg");
    const double legacy_translation_sigma_m =
        required<double>(detector, "loop_translation_sigma_m");
    result.std_detection.loop_yaw_sigma_rad =
        optional<double>(detector, "loop_yaw_sigma_deg",
                         legacy_rotation_sigma_deg) * kRadiansPerDegree;
    result.std_detection.loop_roll_pitch_sigma_rad =
        optional<double>(detector, "loop_roll_pitch_sigma_deg", 30.0) *
        kRadiansPerDegree;
    result.std_detection.loop_xy_translation_sigma_m =
        optional<double>(detector, "loop_xy_translation_sigma_m",
                         legacy_translation_sigma_m);
    result.std_detection.loop_z_translation_sigma_m =
        optional<double>(detector, "loop_z_translation_sigma_m", 5.0);
    result.std_detection.maximum_gravity_disagreement_rad =
        optional<double>(detector, "maximum_gravity_disagreement_deg", 25.0) *
        kRadiansPerDegree;
    result.std_detection.refinement_iterations =
        required_size(detector, "refinement_iterations");
    result.std_detection.required_consistent_detections =
        required_size(detector, "required_consistent_detections");
    result.std_detection.single_detection_overlap_minimum =
        required<double>(detector, "single_detection_overlap_minimum");
    result.std_detection.accepted_loop_cooldown_s =
        required<double>(detector, "accepted_loop_cooldown_s");
    result.std_detection.maximum_loops_per_revisit =
        optional<std::size_t>(detector, "maximum_loops_per_revisit", 0U);
    result.std_detection.revisit_exit_distance_m =
        optional<double>(detector, "revisit_exit_distance_m", 0.0);
    result.std_detection.confirmation_translation_m =
        required<double>(detector, "confirmation_translation_m");
    result.std_detection.confirmation_rotation_rad =
        required<double>(detector, "confirmation_rotation_deg") * kRadiansPerDegree;
    result.std_detection.confirmation_target_id_tolerance =
        required_size(detector, "confirmation_target_id_tolerance");
    result.std_detection.maximum_pose_distance_m =
        required<double>(detector, "maximum_pose_distance_m");
    result.std_detection.maximum_prior_translation_error_m =
        required<double>(detector, "maximum_prior_translation_error_m");
    result.std_detection.maximum_prior_rotation_rad =
        required<double>(detector, "maximum_prior_rotation_error_deg") *
        kRadiansPerDegree;
    return result;
}

}  // namespace uwfl2::loop_closure

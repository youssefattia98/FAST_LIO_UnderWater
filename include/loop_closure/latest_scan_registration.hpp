#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "loop_closure/shadow_map.hpp"

namespace uwfl2::loop_closure
{

struct LatestScanSnapshot
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double timestamp = 0.0;
    std::uint64_t scan_generation = 0;
    std::uint64_t active_tree_generation = 0;
    Pose3d T_local_vehicle_raw;
    Pose3d T_vehicle_sonar;
    std::shared_ptr<const std::vector<PointXYZI>> sonar_points;
};

struct RegistrationConfig
{
    int maximum_iterations = 12;
    std::size_t minimum_effective_points = 30;
    double maximum_neighbor_distance_m = 2.25;
    double plane_fit_threshold_m = 0.12;
    double robust_huber_delta_m = 0.15;
    double convergence_translation_m = 1e-4;
    double convergence_rotation_rad = 1e-4;
    double minimum_information_eigenvalue = 1e-5;
    double maximum_information_condition = 1e10;
    double maximum_registration_translation_m = 2.0;
    double maximum_registration_rotation_rad =
        15.0 * 3.14159265358979323846 / 180.0;
};

struct RegistrationResult
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid = false;
    bool converged = false;
    std::string reason;
    std::uint64_t graph_version = 0;
    std::uint64_t shadow_tree_generation = 0;
    std::uint64_t scan_generation = 0;
    std::uint64_t source_active_tree_generation = 0;
    double scan_timestamp = 0.0;
    Pose3d T_local_vehicle_raw;
    Pose3d T_local_vehicle_initial;
    Pose3d T_local_vehicle_registered;
    Matrix6d covariance = Matrix6d::Identity();
    std::size_t initial_effective_points = 0;
    std::size_t final_effective_points = 0;
    double initial_residual_mean_m = 0.0;
    double initial_residual_p95_m = 0.0;
    double final_residual_mean_m = 0.0;
    double final_residual_p95_m = 0.0;
    double information_min_eigenvalue = 0.0;
    double information_condition = 0.0;
    int iterations = 0;
    double registration_time_ms = 0.0;
};

class LatestScanRegistrar
{
public:
    explicit LatestScanRegistrar(RegistrationConfig config = {});

    RegistrationResult register_scan(
        const LatestScanSnapshot &scan,
        const PoseGraphSnapshot &graph,
        const ShadowMapResult &shadow_map) const;

private:
    RegistrationConfig config_;
};

}  // namespace uwfl2::loop_closure

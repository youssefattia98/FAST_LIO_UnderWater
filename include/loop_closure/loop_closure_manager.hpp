#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include "loop_closure/bounded_queue.hpp"
#include "loop_closure/latest_scan_registration.hpp"
#include "loop_closure/loop_closure_types.hpp"
#include "loop_closure/pose_graph.hpp"
#include "loop_closure/shadow_map.hpp"
#include "loop_closure/std_detector.hpp"

namespace uwfl2::loop_closure
{

struct LoopClosureConfig
{
    bool enabled = false;
    bool automatic_detection_enabled = false;
    KeyframeSelectionConfig keyframes;
    PoseGraphConfig pose_graph;
    ShadowMapConfig shadow_map;
    RegistrationConfig registration;
    StdConfig std_detection;
    std::size_t queue_capacity = 8;
    std::filesystem::path diagnostics_directory;
};

struct LoopClosureStats
{
    std::uint64_t submitted = 0;
    std::uint64_t processed = 0;
    std::uint64_t failed = 0;
    std::uint64_t dropped = 0;
    std::uint64_t graph_version = 0;
    std::uint64_t loops_accepted = 0;
    std::uint64_t loops_rejected = 0;
    std::uint64_t shadow_builds_started = 0;
    std::uint64_t shadow_builds_ready = 0;
    std::uint64_t shadow_builds_failed = 0;
    std::uint64_t shadow_builds_stale = 0;
    std::uint64_t shadow_builds_superseded = 0;
    double shadow_build_time_ms_sum = 0.0;
    double shadow_build_time_ms_max = 0.0;
    std::uint64_t registrations_ready = 0;
    std::uint64_t registrations_rejected = 0;
    double registration_time_ms_sum = 0.0;
    double registration_time_ms_max = 0.0;
    std::uint64_t corrections_committed = 0;
    std::uint64_t corrections_rejected = 0;
    double commit_time_ms_sum = 0.0;
    double commit_time_ms_max = 0.0;
    double graph_time_ms_sum = 0.0;
    double graph_time_ms_max = 0.0;
    std::uint64_t std_processed = 0;
    std::uint64_t std_proposed = 0;
    std::uint64_t std_descriptor_rejected = 0;
    std::uint64_t std_geometry_rejected = 0;
    std::uint64_t std_awaiting_confirmation = 0;
    std::uint64_t std_graph_rejected = 0;
    std::uint64_t std_accepted = 0;
};

struct PendingCorrection
{
    std::shared_ptr<const ShadowMapResult> shadow_map;
    std::shared_ptr<const RegistrationResult> registration;
    std::shared_ptr<const LatestScanSnapshot> scan;
};

class LoopClosureManager
{
public:
    explicit LoopClosureManager(LoopClosureConfig config);
    ~LoopClosureManager();

    LoopClosureManager(const LoopClosureManager &) = delete;
    LoopClosureManager &operator=(const LoopClosureManager &) = delete;

    template <typename PointRange>
    bool try_submit(
        double timestamp,
        const Pose3d &T_local_vehicle,
        const Matrix6d &pose_covariance,
        const Pose3d &T_vehicle_sonar,
        const PointRange &points,
        std::uint64_t tree_generation)
    {
        if (!config_.enabled ||
            !selector_.should_select(timestamp, T_local_vehicle, points.size()))
        {
            return false;
        }
        if (!covariance_is_valid(pose_covariance) || !T_vehicle_sonar.finite())
        {
            ++failed_;
            return false;
        }

        auto local_points = std::make_shared<std::vector<PointXYZI>>();
        local_points->reserve(points.size());
        for (const auto &point : points)
        {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
            {
                local_points->push_back(
                    {point.x, point.y, point.z, point.intensity});
            }
        }
        if (local_points->size() < config_.keyframes.minimum_points)
        {
            ++failed_;
            return false;
        }

        Keyframe keyframe;
        keyframe.id = next_keyframe_id_++;
        keyframe.timestamp = timestamp;
        keyframe.T_local_vehicle = T_local_vehicle.normalized();
        keyframe.pose_covariance = pose_covariance;
        keyframe.T_vehicle_sonar = T_vehicle_sonar.normalized();
        keyframe.sonar_points = std::move(local_points);
        keyframe.tree_generation = tree_generation;

        selector_.accept(timestamp, T_local_vehicle);
        ++submitted_;
        if (!queue_.push_latest(std::move(keyframe)))
        {
            ++failed_;
            return false;
        }
        return true;
    }

    LoopClosureStats stats() const;
    PoseGraphSnapshot graph_snapshot() const;
    LoopEvaluation inject_loop(const LoopConstraint &constraint);
    void notify_active_tree_generation(std::uint64_t generation);
    std::shared_ptr<const ShadowMapResult> shadow_map_snapshot() const;
    std::shared_ptr<const RegistrationResult> registration_snapshot() const;
    std::shared_ptr<const PendingCorrection> take_pending_correction();
    void notify_correction_result(bool committed, double elapsed_ms,
                                  const std::string &reason);

    template <typename PointRange>
    void notify_latest_scan(double timestamp,
                            std::uint64_t scan_generation,
                            std::uint64_t tree_generation,
                            const Pose3d &T_local_vehicle,
                            const Pose3d &T_vehicle_sonar,
                            const PointRange &points)
    {
        if (!std::isfinite(timestamp) || !T_local_vehicle.finite() ||
            !T_vehicle_sonar.finite())
        {
            return;
        }
        auto copied = std::make_shared<std::vector<PointXYZI>>();
        copied->reserve(points.size());
        for (const auto &point : points)
        {
            if (std::isfinite(point.x) && std::isfinite(point.y) &&
                std::isfinite(point.z))
            {
                copied->push_back({point.x, point.y, point.z, point.intensity});
            }
        }
        if (copied->empty())
        {
            return;
        }
        auto scan = std::make_shared<LatestScanSnapshot>();
        scan->timestamp = timestamp;
        scan->scan_generation = scan_generation;
        scan->active_tree_generation = tree_generation;
        scan->T_local_vehicle_raw = T_local_vehicle.normalized();
        scan->T_vehicle_sonar = T_vehicle_sonar.normalized();
        scan->sonar_points = std::move(copied);
        std::atomic_store_explicit(
            &latest_scan_, std::shared_ptr<const LatestScanSnapshot>(scan),
            std::memory_order_release);
    }

private:
    void run();
    void write_diagnostic(const Keyframe &keyframe, std::uint64_t version,
                          double graph_time_ms);
    void write_summary() const;
    void write_loop_diagnostic(const LoopConstraint &constraint,
                               const LoopEvaluation &evaluation);
    void run_shadow_builder();
    void write_shadow_diagnostic(const ShadowMapResult &result,
                                 const std::string &status);
    void write_registration_diagnostic(const RegistrationResult &result);
    void write_std_diagnostic(const StdDetectionResult &result,
                              const LoopEvaluation *evaluation);

    LoopClosureConfig config_;
    KeyframeSelector selector_;
    BoundedQueue<Keyframe> queue_;
    BoundedQueue<ShadowMapRequest> shadow_queue_{1};
    FullSe3PoseGraph pose_graph_;
    ShadowMapBuilder shadow_map_builder_;
    LatestScanRegistrar registrar_;
    std::unique_ptr<StableTriangleDetector> std_detector_;
    std::thread worker_;
    std::thread shadow_worker_;
    std::ofstream diagnostics_;
    std::ofstream loop_diagnostics_;
    std::ofstream shadow_diagnostics_;
    std::ofstream registration_diagnostics_;
    std::ofstream std_diagnostics_;
    std::ofstream commit_diagnostics_;
    mutable std::mutex diagnostics_mutex_;
    mutable std::mutex shadow_result_mutex_;
    std::shared_ptr<const ShadowMapResult> latest_shadow_map_;
    std::shared_ptr<const RegistrationResult> latest_registration_;
    std::shared_ptr<const LatestScanSnapshot> latest_scan_;
    std::shared_ptr<const PendingCorrection> pending_correction_;
    std::uint64_t next_keyframe_id_ = 0;
    std::atomic<std::uint64_t> submitted_{0};
    std::atomic<std::uint64_t> processed_{0};
    std::atomic<std::uint64_t> failed_{0};
    std::atomic<std::uint64_t> graph_version_{0};
    std::atomic<std::uint64_t> active_tree_generation_{0};
    std::atomic<std::uint64_t> loops_accepted_{0};
    std::atomic<std::uint64_t> loops_rejected_{0};
    std::atomic<std::uint64_t> shadow_builds_started_{0};
    std::atomic<std::uint64_t> shadow_builds_ready_{0};
    std::atomic<std::uint64_t> shadow_builds_failed_{0};
    std::atomic<std::uint64_t> shadow_builds_stale_{0};
    std::atomic<double> shadow_build_time_ms_sum_{0.0};
    std::atomic<double> shadow_build_time_ms_max_{0.0};
    std::atomic<std::uint64_t> registrations_ready_{0};
    std::atomic<std::uint64_t> registrations_rejected_{0};
    std::atomic<double> registration_time_ms_sum_{0.0};
    std::atomic<double> registration_time_ms_max_{0.0};
    std::atomic<std::uint64_t> corrections_committed_{0};
    std::atomic<std::uint64_t> corrections_rejected_{0};
    std::atomic<double> commit_time_ms_sum_{0.0};
    std::atomic<double> commit_time_ms_max_{0.0};
    std::atomic<double> graph_time_ms_sum_{0.0};
    std::atomic<double> graph_time_ms_max_{0.0};
    std::atomic<std::uint64_t> std_processed_{0};
    std::atomic<std::uint64_t> std_proposed_{0};
    std::atomic<std::uint64_t> std_descriptor_rejected_{0};
    std::atomic<std::uint64_t> std_geometry_rejected_{0};
    std::atomic<std::uint64_t> std_awaiting_confirmation_{0};
    std::atomic<std::uint64_t> std_graph_rejected_{0};
    std::atomic<std::uint64_t> std_accepted_{0};
};

}  // namespace uwfl2::loop_closure

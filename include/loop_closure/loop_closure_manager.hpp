#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <thread>
#include <utility>

#include "loop_closure/bounded_queue.hpp"
#include "loop_closure/loop_closure_types.hpp"
#include "loop_closure/pose_graph.hpp"

namespace uwfl2::loop_closure
{

struct LoopClosureConfig
{
    bool enabled = false;
    bool automatic_detection_enabled = false;
    KeyframeSelectionConfig keyframes;
    PoseGraphConfig pose_graph;
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
    double graph_time_ms_sum = 0.0;
    double graph_time_ms_max = 0.0;
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

private:
    void run();
    void write_diagnostic(const Keyframe &keyframe, std::uint64_t version,
                          double graph_time_ms);
    void write_summary() const;

    LoopClosureConfig config_;
    KeyframeSelector selector_;
    BoundedQueue<Keyframe> queue_;
    FullSe3PoseGraph pose_graph_;
    std::thread worker_;
    std::ofstream diagnostics_;
    std::uint64_t next_keyframe_id_ = 0;
    std::atomic<std::uint64_t> submitted_{0};
    std::atomic<std::uint64_t> processed_{0};
    std::atomic<std::uint64_t> failed_{0};
    std::atomic<std::uint64_t> graph_version_{0};
    std::atomic<double> graph_time_ms_sum_{0.0};
    std::atomic<double> graph_time_ms_max_{0.0};
};

}  // namespace uwfl2::loop_closure

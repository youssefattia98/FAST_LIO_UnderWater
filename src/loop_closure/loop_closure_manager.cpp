#include "loop_closure/loop_closure_manager.hpp"

#include <algorithm>
#include <stdexcept>

namespace uwfl2::loop_closure
{
LoopClosureManager::LoopClosureManager(LoopClosureConfig config)
    : config_(std::move(config)),
      selector_(config_.keyframes),
      queue_(config_.queue_capacity),
      pose_graph_(config_.pose_graph),
      shadow_map_builder_(config_.shadow_map),
      registrar_(config_.registration)
{
    if (!config_.enabled)
    {
        throw std::invalid_argument(
            "LoopClosureManager must only be constructed when enabled");
    }
    if (config_.automatic_detection_enabled)
    {
        std_detector_ = std::make_unique<StableTriangleDetector>(
            config_.std_detection);
    }
    worker_ = std::thread(&LoopClosureManager::run, this);
    shadow_worker_ = std::thread(&LoopClosureManager::run_shadow_builder, this);
    registration_worker_ =
        std::thread(&LoopClosureManager::run_registration_refresher, this);
}

LoopEvaluation LoopClosureManager::process_loop(const LoopConstraint &constraint)
{
    LoopEvaluation result = pose_graph_.try_add_loop(constraint);
    graph_version_.store(result.graph_version);
    if (result.accepted)
    {
        ++loops_accepted_;
        const PoseGraphSnapshot graph = pose_graph_.snapshot();
        ShadowMapRequest request;
        request.graph = graph;
        request.source_tree_generation = active_tree_generation_.load();
        shadow_queue_.push_latest(std::move(request));
    }
    else
    {
        ++loops_rejected_;
    }
    record_visualization_event(
        {constraint.to_id, constraint.from_id, result.accepted,
         !result.accepted, result.reason});
    return result;
}

void LoopClosureManager::notify_active_tree_generation(std::uint64_t generation)
{
    active_tree_generation_.store(generation);
}

LoopClosureManager::~LoopClosureManager()
{
    queue_.close();
    if (worker_.joinable())
    {
        worker_.join();
    }
    shadow_queue_.close();
    if (shadow_worker_.joinable())
    {
        shadow_worker_.join();
    }
    registration_queue_.close();
    if (registration_worker_.joinable())
    {
        registration_worker_.join();
    }
}

void LoopClosureManager::run_shadow_builder()
{
    while (const std::optional<ShadowMapRequest> request = shadow_queue_.wait_pop())
    {
        ++shadow_builds_started_;
        ShadowMapResult result = shadow_map_builder_.build(*request);
        const double elapsed_ms = result.total_time_ms;
        double sum = shadow_build_time_ms_sum_.load();
        while (!shadow_build_time_ms_sum_.compare_exchange_weak(sum, sum + elapsed_ms))
        {
        }
        double maximum = shadow_build_time_ms_max_.load();
        while (maximum < elapsed_ms &&
               !shadow_build_time_ms_max_.compare_exchange_weak(maximum, elapsed_ms))
        {
        }
        if (!result.valid)
        {
            ++shadow_builds_failed_;
            continue;
        }
        PoseGraphSnapshot current_graph = pose_graph_.snapshot();
        const std::uint64_t current_graph_version = current_graph.version;
        if (!shadow_result_matches_graph_version(result, current_graph_version))
        {
            ++shadow_builds_stale_;
            // Keyframes may arrive while an asynchronous rebuild is running.
            // Requeue the newest graph snapshot so a valid accepted loop is not
            // lost merely because the front end advanced by one keyframe.
            shadow_queue_.push_latest(ShadowMapRequest{
                std::move(current_graph), active_tree_generation_.load()});
            continue;
        }
        auto shadow_map =
            std::make_shared<const ShadowMapResult>(std::move(result));
        ++shadow_builds_ready_;
        registration_queue_.push_latest(
            RegistrationRefreshRequest{shadow_map, request->graph, 0});
    }
}

void LoopClosureManager::run_registration_refresher()
{
    while (const std::optional<RegistrationRefreshRequest> request =
               registration_queue_.wait_pop())
    {
        const auto scan = std::atomic_load_explicit(
            &latest_scan_, std::memory_order_acquire);
        if (!scan || !request->shadow_map ||
            !shadow_result_matches_graph(*request->shadow_map, request->graph))
        {
            ++registrations_rejected_;
            continue;
        }
        RegistrationResult registration = registrar_.register_scan(
            *scan, request->graph, *request->shadow_map);
        double sum_registration = registration_time_ms_sum_.load();
        while (!registration_time_ms_sum_.compare_exchange_weak(
            sum_registration, sum_registration + registration.registration_time_ms))
        {
        }
        double max_registration = registration_time_ms_max_.load();
        while (max_registration < registration.registration_time_ms &&
               !registration_time_ms_max_.compare_exchange_weak(
                   max_registration, registration.registration_time_ms))
        {
        }
        if (!registration.valid)
        {
            ++registrations_rejected_;
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(shadow_result_mutex_);
            latest_shadow_map_ = request->shadow_map;
            latest_registration_ =
                std::make_shared<const RegistrationResult>(std::move(registration));
            auto pending = std::make_shared<PendingCorrection>();
            pending->shadow_map = latest_shadow_map_;
            pending->registration = latest_registration_;
            pending->scan = scan;
            pending->refresh_attempt = request->attempt;
            pending_correction_ = std::move(pending);
        }
        ++registrations_ready_;
    }
}

bool LoopClosureManager::request_registration_refresh(
    const std::shared_ptr<const ShadowMapResult> &shadow_map,
    std::size_t attempt)
{
    constexpr std::size_t maximum_attempts = 4;
    if (!shadow_map || attempt > maximum_attempts)
    {
        return false;
    }
    PoseGraphSnapshot graph = pose_graph_.snapshot();
    if (!shadow_result_matches_graph(*shadow_map, graph))
    {
        return false;
    }
    return registration_queue_.push_latest(
        RegistrationRefreshRequest{shadow_map, std::move(graph), attempt});
}

void LoopClosureManager::run()
{
    while (const std::optional<Keyframe> keyframe = queue_.wait_pop())
    {
        const auto started = std::chrono::steady_clock::now();
        try
        {
            const std::uint64_t version = pose_graph_.append_keyframe(*keyframe);
            const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                          std::chrono::steady_clock::now() - started)
                                          .count();
            graph_version_.store(version);
            double sum = graph_time_ms_sum_.load();
            while (!graph_time_ms_sum_.compare_exchange_weak(sum, sum + elapsed_ms))
            {
            }
            double maximum = graph_time_ms_max_.load();
            while (maximum < elapsed_ms &&
                   !graph_time_ms_max_.compare_exchange_weak(maximum, elapsed_ms))
            {
            }
            ++processed_;

            if (std_detector_)
            {
                const StdDetectionResult detection = std_detector_->process(*keyframe);
                ++std_processed_;
                if (detection.proposed)
                {
                    ++std_proposed_;
                }
                if (detection.reason == "descriptor_rejected")
                {
                    ++std_descriptor_rejected_;
                }
                else if (detection.reason == "geometry_rejected")
                {
                    ++std_geometry_rejected_;
                }
                else if (detection.reason == "awaiting_confirmation")
                {
                    ++std_awaiting_confirmation_;
                }

                if (detection.proposed && !detection.loop)
                {
                    record_visualization_event(
                        {detection.source_id, detection.target_id, false, false,
                         detection.reason});
                }

                if (detection.loop)
                {
                    const LoopEvaluation evaluation = process_loop(*detection.loop);
                    if (evaluation.accepted)
                    {
                        ++std_accepted_;
                        std_detector_->notify_loop_accepted(
                            detection.source_id, detection.target_id,
                            keyframe->timestamp);
                    }
                    else
                    {
                        ++std_graph_rejected_;
                    }
                }
            }
        }
        catch (const std::exception &)
        {
            ++failed_;
        }
    }
}

void LoopClosureManager::record_visualization_event(
    const LoopVisualizationEvent &event)
{
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    visualization_events_.push_back(event);
    constexpr std::size_t maximum_events = 32;
    while (visualization_events_.size() > maximum_events)
    {
        visualization_events_.pop_front();
    }
}

std::vector<LoopVisualizationEvent>
LoopClosureManager::visualization_events() const
{
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    return {visualization_events_.begin(), visualization_events_.end()};
}

LoopClosureStats LoopClosureManager::stats() const
{
    LoopClosureStats result;
    result.submitted = submitted_.load();
    result.processed = processed_.load();
    result.failed = failed_.load();
    result.dropped = queue_.dropped();
    result.graph_version = graph_version_.load();
    result.loops_accepted = loops_accepted_.load();
    result.loops_rejected = loops_rejected_.load();
    result.shadow_builds_started = shadow_builds_started_.load();
    result.shadow_builds_ready = shadow_builds_ready_.load();
    result.shadow_builds_failed = shadow_builds_failed_.load();
    result.shadow_builds_stale = shadow_builds_stale_.load();
    result.shadow_builds_superseded = shadow_queue_.dropped();
    result.shadow_build_time_ms_sum = shadow_build_time_ms_sum_.load();
    result.shadow_build_time_ms_max = shadow_build_time_ms_max_.load();
    result.registrations_ready = registrations_ready_.load();
    result.registrations_rejected = registrations_rejected_.load();
    result.registration_time_ms_sum = registration_time_ms_sum_.load();
    result.registration_time_ms_max = registration_time_ms_max_.load();
    result.corrections_committed = corrections_committed_.load();
    result.corrections_rejected = corrections_rejected_.load();
    result.commit_time_ms_sum = commit_time_ms_sum_.load();
    result.commit_time_ms_max = commit_time_ms_max_.load();
    result.graph_time_ms_sum = graph_time_ms_sum_.load();
    result.graph_time_ms_max = graph_time_ms_max_.load();
    result.std_processed = std_processed_.load();
    result.std_proposed = std_proposed_.load();
    result.std_descriptor_rejected = std_descriptor_rejected_.load();
    result.std_geometry_rejected = std_geometry_rejected_.load();
    result.std_awaiting_confirmation = std_awaiting_confirmation_.load();
    result.std_graph_rejected = std_graph_rejected_.load();
    result.std_accepted = std_accepted_.load();
    return result;
}

std::shared_ptr<const RegistrationResult>
LoopClosureManager::registration_snapshot() const
{
    std::lock_guard<std::mutex> lock(shadow_result_mutex_);
    return latest_registration_;
}

std::shared_ptr<const PendingCorrection>
LoopClosureManager::take_pending_correction()
{
    std::lock_guard<std::mutex> lock(shadow_result_mutex_);
    auto result = pending_correction_;
    pending_correction_.reset();
    return result;
}

void LoopClosureManager::notify_correction_result(
    bool committed, double elapsed_ms)
{
    if (committed)
    {
        ++corrections_committed_;
    }
    else
    {
        ++corrections_rejected_;
    }
    double sum = commit_time_ms_sum_.load();
    while (!commit_time_ms_sum_.compare_exchange_weak(sum, sum + elapsed_ms))
    {
    }
    double maximum = commit_time_ms_max_.load();
    while (maximum < elapsed_ms &&
           !commit_time_ms_max_.compare_exchange_weak(maximum, elapsed_ms))
    {
    }
}

PoseGraphSnapshot LoopClosureManager::graph_snapshot() const
{
    return pose_graph_.snapshot();
}

}  // namespace uwfl2::loop_closure

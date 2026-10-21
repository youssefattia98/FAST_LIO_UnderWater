#include "loop_closure/loop_closure_manager.hpp"

#include <algorithm>
#include <stdexcept>

namespace uwfl2::loop_closure
{
namespace
{
void add_sample(std::atomic<double> &sum, std::atomic<double> &maximum,
                double value)
{
    double current_sum = sum.load();
    while (!sum.compare_exchange_weak(current_sum, current_sum + value))
    {
    }
    double current_maximum = maximum.load();
    while (current_maximum < value &&
           !maximum.compare_exchange_weak(current_maximum, value))
    {
    }
}
}  // namespace

LoopClosureManager::LoopClosureManager(LoopClosureConfig config)
    : config_(std::move(config)),
      selector_(config_.keyframes),
      queue_(config_.queue_capacity),
      detector_queue_(config_.queue_capacity),
      scan_history_capacity_(std::max<std::size_t>(
          32U, config_.queue_capacity * 4U)),
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
    detector_worker_ = std::thread(&LoopClosureManager::run_detector, this);
    shadow_worker_ = std::thread(&LoopClosureManager::run_shadow_builder, this);
    registration_worker_ =
        std::thread(&LoopClosureManager::run_registration_refresher, this);
}

LoopEvaluation LoopClosureManager::process_loop(const LoopConstraint &constraint)
{
    {
        std::lock_guard<std::mutex> lock(transaction_mutex_);
        if (active_transaction_)
        {
            LoopEvaluation busy;
            busy.reason = "candidate_in_flight";
            busy.graph_version = graph_version_.load();
            {
                std::lock_guard<std::mutex> diagnostic_lock(
                    registration_diagnostic_mutex_);
                last_graph_evaluation_ = busy;
            }
            ++loops_rejected_;
            record_visualization_event(
                {constraint.to_id, constraint.from_id, false, true,
                 busy.reason});
            return busy;
        }
    }

    EvaluatedLoopCandidate evaluated = pose_graph_.evaluate_loop(constraint);
    LoopEvaluation result = evaluated.candidate.evaluation;
    {
        std::lock_guard<std::mutex> diagnostic_lock(
            registration_diagnostic_mutex_);
        last_graph_evaluation_ = result;
    }
    if (result.accepted)
    {
        ++loops_accepted_;
        auto transaction = std::make_shared<LoopTransaction>();
        transaction->id = next_transaction_id_++;
        transaction->evaluated = std::move(evaluated);
        transaction->detected_at = std::chrono::steady_clock::now();
        const auto source = std::find(
            transaction->evaluated.graph.ids.begin(),
            transaction->evaluated.graph.ids.end(), constraint.to_id);
        if (source != transaction->evaluated.graph.ids.end())
        {
            const std::size_t index = static_cast<std::size_t>(std::distance(
                transaction->evaluated.graph.ids.begin(), source));
            transaction->source_timestamp =
                transaction->evaluated.graph.timestamps[index];
        }
        {
            std::lock_guard<std::mutex> lock(transaction_mutex_);
            active_transaction_ = transaction;
            active_transaction_stage_ = "shadow_building";
        }
        ShadowMapRequest request;
        request.graph = transaction->evaluated.graph;
        request.source_tree_generation = active_tree_generation_.load();
        request.transaction_id = transaction->id;
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
    const auto current_scan = std::atomic_load_explicit(
        &latest_scan_, std::memory_order_acquire);
    if (!current_scan || current_scan->active_tree_generation == generation)
    {
        return;
    }

    // An active-map insertion does not change the retained raw sonar
    // measurement or its pose.  Retag the immutable snapshot so a sparse tail
    // scan cannot leave an otherwise registration-capable scan permanently
    // stale with respect to the live tree.
    auto updated_scan = std::make_shared<LatestScanSnapshot>(*current_scan);
    updated_scan->active_tree_generation = generation;
    std::atomic_store_explicit(
        &latest_scan_, std::shared_ptr<const LatestScanSnapshot>(updated_scan),
        std::memory_order_release);
}

LoopClosureManager::~LoopClosureManager()
{
    queue_.close();
    if (worker_.joinable())
    {
        worker_.join();
    }
    detector_queue_.close();
    if (detector_worker_.joinable())
    {
        detector_worker_.join();
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
            std::lock_guard<std::mutex> lock(transaction_mutex_);
            if (active_transaction_ &&
                active_transaction_->id == result.transaction_id)
            {
                active_transaction_.reset();
            }
            continue;
        }
        std::shared_ptr<const LoopTransaction> transaction;
        {
            std::lock_guard<std::mutex> lock(transaction_mutex_);
            if (active_transaction_ &&
                active_transaction_->id == result.transaction_id)
            {
                transaction = active_transaction_;
            }
        }
        if (!transaction ||
            result.graph_loop_factor_count !=
                transaction->evaluated.graph.loop_factor_count)
        {
            ++shadow_builds_stale_;
            continue;
        }
        auto shadow_map =
            std::make_shared<const ShadowMapResult>(std::move(result));
        ++shadow_builds_ready_;
        {
            std::lock_guard<std::mutex> lock(transaction_mutex_);
            if (active_transaction_ && active_transaction_->id == transaction->id)
            {
                active_transaction_stage_ = "registration";
            }
        }
        const auto registration_scan = registration_scan_for(transaction);
        registration_queue_.push_latest(
            RegistrationRefreshRequest{
                shadow_map, transaction->evaluated.graph, transaction,
                registration_scan, 0});
    }
}

std::shared_ptr<const LatestScanSnapshot>
LoopClosureManager::registration_scan_for(
    const std::shared_ptr<const LoopTransaction> &transaction) const
{
    if (!transaction)
    {
        return {};
    }
    std::deque<std::shared_ptr<const LatestScanSnapshot>> history;
    {
        std::lock_guard<std::mutex> lock(scan_history_mutex_);
        history = scan_history_;
    }
    auto selected = first_registration_scan_after(
        history, transaction->source_timestamp);
    if (selected)
    {
        return selected;
    }

    // A loop may be detected on the final usable scan of a live mission.  In
    // that case there is no later event to validate with, so use the exact
    // source keyframe rather than a scheduler-dependent global latest scan.
    const auto &graph = transaction->evaluated.graph;
    const auto source = std::find(
        graph.ids.begin(), graph.ids.end(),
        transaction->evaluated.candidate.constraint.to_id);
    if (source == graph.ids.end())
    {
        return {};
    }
    const std::size_t index = static_cast<std::size_t>(
        std::distance(graph.ids.begin(), source));
    if (index >= graph.keyframes.size())
    {
        return {};
    }
    const Keyframe &keyframe = graph.keyframes[index];
    if (!keyframe.sonar_points || keyframe.sonar_points->empty())
    {
        return {};
    }
    auto fallback = std::make_shared<LatestScanSnapshot>();
    fallback->timestamp = keyframe.timestamp;
    fallback->scan_generation = keyframe.id + 1U;
    fallback->active_tree_generation = keyframe.tree_generation;
    fallback->T_local_vehicle_raw = keyframe.T_local_vehicle;
    fallback->T_vehicle_sonar = keyframe.T_vehicle_sonar;
    fallback->sonar_points = keyframe.sonar_points;
    return fallback;
}

void LoopClosureManager::run_registration_refresher()
{
    while (const std::optional<RegistrationRefreshRequest> request =
               registration_queue_.wait_pop())
    {
        const auto scan = request->scan;
        if (!scan || !request->shadow_map || !request->transaction ||
            !shadow_result_matches_graph(*request->shadow_map, request->graph))
        {
            {
                std::lock_guard<std::mutex> lock(registration_diagnostic_mutex_);
                last_registration_rejection_reason_ =
                    "missing_scan_or_stale_shadow_graph";
            }
            ++registrations_rejected_;
            std::lock_guard<std::mutex> lock(transaction_mutex_);
            if (active_transaction_ && request->transaction &&
                active_transaction_->id == request->transaction->id)
            {
                active_transaction_.reset();
            }
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
            {
                std::lock_guard<std::mutex> lock(registration_diagnostic_mutex_);
                last_registration_rejection_reason_ = registration.reason;
            }
            ++registrations_rejected_;
            if (request->attempt < config_.registration.maximum_refresh_attempts)
            {
                registration_queue_.push_latest(RegistrationRefreshRequest{
                    request->shadow_map, request->graph,
                    request->transaction, request->scan,
                    request->attempt + 1});
            }
            else
            {
                std::lock_guard<std::mutex> lock(transaction_mutex_);
                if (active_transaction_ &&
                    active_transaction_->id == request->transaction->id)
                {
                    active_transaction_.reset();
                }
            }
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
            pending->transaction = request->transaction;
            pending->refresh_attempt = request->attempt;
            pending_correction_ = std::move(pending);
        }
        {
            std::lock_guard<std::mutex> lock(transaction_mutex_);
            if (active_transaction_ &&
                active_transaction_->id == request->transaction->id)
            {
                active_transaction_stage_ = "pending_commit";
            }
        }
        ++registrations_ready_;
    }
}

bool LoopClosureManager::request_registration_refresh(
    const std::shared_ptr<const ShadowMapResult> &shadow_map,
    std::size_t attempt)
{
    if (!shadow_map ||
        attempt > config_.registration.maximum_refresh_attempts)
    {
        return false;
    }
    std::shared_ptr<const LoopTransaction> transaction;
    {
        std::lock_guard<std::mutex> lock(transaction_mutex_);
        transaction = active_transaction_;
    }
    if (!transaction || shadow_map->transaction_id != transaction->id ||
        !shadow_result_matches_graph(
            *shadow_map, transaction->evaluated.graph))
    {
        return false;
    }
    return registration_queue_.push_latest(
        RegistrationRefreshRequest{
            shadow_map, transaction->evaluated.graph, transaction,
            registration_scan_for(transaction), attempt});
}

EvaluatedLoopCandidate LoopClosureManager::commit_loop_candidate(
    const std::shared_ptr<const LoopTransaction> &transaction)
{
    if (!transaction)
    {
        EvaluatedLoopCandidate invalid;
        invalid.candidate.evaluation.reason = "missing_loop_transaction";
        return invalid;
    }
    {
        std::lock_guard<std::mutex> lock(transaction_mutex_);
        if (!active_transaction_ || active_transaction_->id != transaction->id)
        {
            EvaluatedLoopCandidate stale;
            stale.candidate = transaction->evaluated.candidate;
            stale.candidate.evaluation.accepted = false;
            stale.candidate.evaluation.reason = "inactive_loop_transaction";
            return stale;
        }
    }
    EvaluatedLoopCandidate committed =
        pose_graph_.commit_loop(transaction->evaluated.candidate);
    if (committed.candidate.evaluation.accepted)
    {
        graph_version_.store(committed.candidate.evaluation.graph_version);
    }
    return committed;
}

void LoopClosureManager::run()
{
    while (std::optional<QueuedKeyframe> queued = queue_.wait_pop())
    {
        const double queue_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - queued->enqueued_at).count();
        ++graph_queue_samples_;
        add_sample(graph_queue_time_ms_sum_, graph_queue_time_ms_max_, queue_ms);
        const auto started = std::chrono::steady_clock::now();
        try
        {
            const std::uint64_t version =
                pose_graph_.append_keyframe(queued->keyframe);
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
            if (std_detector_ &&
                !detector_queue_.push_latest(
                    QueuedKeyframe{std::move(queued->keyframe),
                                   std::chrono::steady_clock::now()}))
            {
                ++failed_;
            }
        }
        catch (const std::exception &)
        {
            ++failed_;
        }
    }
}

void LoopClosureManager::run_detector()
{
    while (std::optional<QueuedKeyframe> queued = detector_queue_.wait_pop())
    {
        const double queue_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - queued->enqueued_at).count();
        ++detector_queue_samples_;
        add_sample(detector_queue_time_ms_sum_, detector_queue_time_ms_max_,
                   queue_ms);
        try
        {
            StdDetectionResult detection;
            {
                std::lock_guard<std::mutex> lock(detector_mutex_);
                const auto optimized =
                    detector_optimized_poses_.find(queued->keyframe.id);
                if (optimized != detector_optimized_poses_.end())
                {
                    // A keyframe can already be waiting in this queue when a
                    // loop commits. Keep it in the synchronized graph frame.
                    queued->keyframe.T_local_vehicle = optimized->second;
                }
                detection = std_detector_->process(queued->keyframe);
            }
            {
                std::lock_guard<std::mutex> lock(detector_diagnostic_mutex_);
                last_std_detection_ = detection;
            }
            add_sample(descriptor_time_ms_sum_, descriptor_time_ms_max_,
                       detection.descriptor_time_ms);
            add_sample(search_time_ms_sum_, search_time_ms_max_,
                       detection.search_time_ms);
            add_sample(verification_time_ms_sum_, verification_time_ms_max_,
                       detection.verification_time_ms);
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
                }
                else
                {
                    ++std_graph_rejected_;
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
    result.detector_dropped = detector_queue_.dropped();
    result.graph_queue_depth = queue_.size();
    result.detector_queue_depth = detector_queue_.size();
    result.graph_queue_samples = graph_queue_samples_.load();
    result.graph_queue_time_ms_sum = graph_queue_time_ms_sum_.load();
    result.graph_queue_time_ms_max = graph_queue_time_ms_max_.load();
    result.detector_queue_samples = detector_queue_samples_.load();
    result.detector_queue_time_ms_sum = detector_queue_time_ms_sum_.load();
    result.detector_queue_time_ms_max = detector_queue_time_ms_max_.load();
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
    {
        std::lock_guard<std::mutex> lock(registration_diagnostic_mutex_);
        result.last_registration_rejection_reason =
            last_registration_rejection_reason_;
    }
    result.registration_time_ms_sum = registration_time_ms_sum_.load();
    result.registration_time_ms_max = registration_time_ms_max_.load();
    result.corrections_committed = corrections_committed_.load();
    result.corrections_rejected = corrections_rejected_.load();
    {
        std::lock_guard<std::mutex> lock(registration_diagnostic_mutex_);
        result.last_commit_rejection_reason = last_commit_rejection_reason_;
        result.last_graph_evaluation = last_graph_evaluation_;
    }
    result.commit_time_ms_sum = commit_time_ms_sum_.load();
    result.commit_time_ms_max = commit_time_ms_max_.load();
    result.candidate_to_commit_time_ms_last =
        candidate_to_commit_time_ms_last_.load();
    result.candidate_to_commit_time_ms_max =
        candidate_to_commit_time_ms_max_.load();
    result.graph_time_ms_sum = graph_time_ms_sum_.load();
    result.graph_time_ms_max = graph_time_ms_max_.load();
    result.std_processed = std_processed_.load();
    result.std_proposed = std_proposed_.load();
    result.std_descriptor_rejected = std_descriptor_rejected_.load();
    result.std_geometry_rejected = std_geometry_rejected_.load();
    result.std_awaiting_confirmation = std_awaiting_confirmation_.load();
    result.std_graph_rejected = std_graph_rejected_.load();
    result.std_accepted = std_accepted_.load();
    {
        std::lock_guard<std::mutex> lock(detector_diagnostic_mutex_);
        result.last_std_detection = last_std_detection_;
    }
    result.descriptor_time_ms_sum = descriptor_time_ms_sum_.load();
    result.descriptor_time_ms_max = descriptor_time_ms_max_.load();
    result.search_time_ms_sum = search_time_ms_sum_.load();
    result.search_time_ms_max = search_time_ms_max_.load();
    result.verification_time_ms_sum = verification_time_ms_sum_.load();
    result.verification_time_ms_max = verification_time_ms_max_.load();
    {
        std::lock_guard<std::mutex> lock(transaction_mutex_);
        if (active_transaction_)
        {
            result.active_transaction_id = active_transaction_->id;
            result.active_transaction_source_id =
                active_transaction_->evaluated.candidate.constraint.to_id;
            result.active_transaction_target_id =
                active_transaction_->evaluated.candidate.constraint.from_id;
            result.active_transaction_age_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() -
                    active_transaction_->detected_at).count();
            result.active_transaction_stage = active_transaction_stage_;
        }
    }
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
    bool committed,
    double elapsed_ms,
    const std::string &rejection_reason,
    const std::optional<Pose3d> &frontend_correction)
{
    std::shared_ptr<const LoopTransaction> completed_transaction;
    if (committed)
    {
        if (frontend_correction)
        {
            selector_.apply_frame_correction(*frontend_correction);
        }
        ++corrections_committed_;
        {
            std::lock_guard<std::mutex> lock(transaction_mutex_);
            completed_transaction = active_transaction_;
            active_transaction_.reset();
            active_transaction_stage_.clear();
        }
        if (completed_transaction && std_detector_)
        {
            const PoseGraphSnapshot graph = pose_graph_.snapshot();
            std::lock_guard<std::mutex> detector_lock(detector_mutex_);
            std_detector_->synchronize_optimized_poses(
                graph.ids, graph.optimized_poses);
            detector_optimized_poses_.clear();
            detector_optimized_poses_.reserve(graph.ids.size());
            for (std::size_t index = 0; index < graph.ids.size(); ++index)
            {
                detector_optimized_poses_.emplace(
                    graph.ids[index], graph.optimized_poses[index]);
            }
            std_detector_->notify_loop_accepted(
                completed_transaction->evaluated.candidate.constraint.to_id,
                completed_transaction->evaluated.candidate.constraint.from_id,
                completed_transaction->source_timestamp);
        }
    }
    else
    {
        {
            std::lock_guard<std::mutex> lock(registration_diagnostic_mutex_);
            last_commit_rejection_reason_ = rejection_reason;
        }
        ++corrections_rejected_;
        std::lock_guard<std::mutex> lock(transaction_mutex_);
        completed_transaction = active_transaction_;
        active_transaction_.reset();
        active_transaction_stage_.clear();
    }
    if (completed_transaction)
    {
        const double transaction_elapsed_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() -
                completed_transaction->detected_at).count();
        candidate_to_commit_time_ms_last_.store(transaction_elapsed_ms);
        double maximum = candidate_to_commit_time_ms_max_.load();
        while (maximum < transaction_elapsed_ms &&
               !candidate_to_commit_time_ms_max_.compare_exchange_weak(
                   maximum, transaction_elapsed_ms))
        {
        }
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

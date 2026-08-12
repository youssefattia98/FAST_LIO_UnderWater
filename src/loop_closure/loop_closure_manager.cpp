#include "loop_closure/loop_closure_manager.hpp"

#include <algorithm>
#include <iomanip>
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
    if (!config_.diagnostics_directory.empty())
    {
        std::filesystem::create_directories(config_.diagnostics_directory);
        diagnostics_.open(config_.diagnostics_directory / "keyframes.csv");
        diagnostics_ << "id,timestamp,tx,ty,tz,qw,qx,qy,qz,points,graph_version,graph_time_ms\n";
        loop_diagnostics_.open(config_.diagnostics_directory / "loops.csv");
        loop_diagnostics_
            << "from_id,to_id,accepted,reason,graph_version,graph_error_before,"
               "graph_error_after,translation_error_before,translation_error_after,"
               "rotation_error_before_rad,rotation_error_after_rad,optimization_time_ms\n";
        shadow_diagnostics_.open(
            config_.diagnostics_directory / "shadow_rebuilds.csv");
        shadow_diagnostics_
            << "graph_version,source_tree_generation,status,reason,selected_keyframes,"
               "input_points,filtered_points,reconstruction_time_ms,downsample_time_ms,"
               "tree_build_time_ms,estimated_tree_bytes\n";
        registration_diagnostics_.open(
            config_.diagnostics_directory / "reregistrations.csv");
        registration_diagnostics_
            << "graph_version,shadow_tree_generation,scan_generation,scan_timestamp,"
               "valid,converged,reason,initial_effective,final_effective,"
               "initial_mean_m,initial_p95_m,final_mean_m,final_p95_m,"
               "information_min_eigenvalue,information_condition,iterations,time_ms\n";
    }
    worker_ = std::thread(&LoopClosureManager::run, this);
    shadow_worker_ = std::thread(&LoopClosureManager::run_shadow_builder, this);
}

LoopEvaluation LoopClosureManager::inject_loop(const LoopConstraint &constraint)
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
    write_loop_diagnostic(constraint, result);
    write_summary();
    return result;
}

void LoopClosureManager::notify_active_tree_generation(std::uint64_t generation)
{
    active_tree_generation_.store(generation);
}

LoopClosureManager::~LoopClosureManager()
{
    queue_.close();
    shadow_queue_.close();
    if (worker_.joinable())
    {
        worker_.join();
    }
    if (shadow_worker_.joinable())
    {
        shadow_worker_.join();
    }
    write_summary();
}

void LoopClosureManager::run_shadow_builder()
{
    while (const std::optional<ShadowMapRequest> request = shadow_queue_.wait_pop())
    {
        ++shadow_builds_started_;
        ShadowMapResult result = shadow_map_builder_.build(*request);
        const double elapsed_ms = result.reconstruction_time_ms +
                                  result.downsample_time_ms +
                                  result.tree_build_time_ms;
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
            write_shadow_diagnostic(result, "failed");
            write_summary();
            continue;
        }
        const std::uint64_t current_graph_version = pose_graph_.snapshot().version;
        if (!shadow_result_matches_graph_version(result, current_graph_version))
        {
            ++shadow_builds_stale_;
            write_shadow_diagnostic(result, "stale");
            write_summary();
            continue;
        }
        const auto scan = std::atomic_load_explicit(
            &latest_scan_, std::memory_order_acquire);
        if (!scan)
        {
            ++registrations_rejected_;
            result.reason = "latest_scan_unavailable";
            write_shadow_diagnostic(result, "registration_rejected");
            write_summary();
            continue;
        }
        RegistrationResult registration =
            registrar_.register_scan(*scan, request->graph, result);
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
        write_registration_diagnostic(registration);
        if (!registration.valid)
        {
            ++registrations_rejected_;
            write_shadow_diagnostic(result, "registration_rejected");
            write_summary();
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(shadow_result_mutex_);
            latest_shadow_map_ =
                std::make_shared<const ShadowMapResult>(std::move(result));
            latest_registration_ =
                std::make_shared<const RegistrationResult>(std::move(registration));
            auto pending = std::make_shared<PendingCorrection>();
            pending->shadow_map = latest_shadow_map_;
            pending->registration = latest_registration_;
            pending->scan = scan;
            pending_correction_ = std::move(pending);
        }
        ++shadow_builds_ready_;
        ++registrations_ready_;
        write_shadow_diagnostic(*shadow_map_snapshot(), "ready");
        write_summary();
    }
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
            write_diagnostic(*keyframe, version, elapsed_ms);
        }
        catch (const std::exception &)
        {
            ++failed_;
        }
    }
}

void LoopClosureManager::write_diagnostic(
    const Keyframe &keyframe,
    std::uint64_t version,
    double graph_time_ms)
{
    if (!diagnostics_)
    {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        const Pose3d pose = keyframe.T_local_vehicle.normalized();
        diagnostics_ << keyframe.id << ',' << std::setprecision(17)
                     << keyframe.timestamp << ',' << pose.translation.x() << ','
                     << pose.translation.y() << ',' << pose.translation.z() << ','
                     << pose.rotation.w() << ',' << pose.rotation.x() << ','
                     << pose.rotation.y() << ',' << pose.rotation.z() << ','
                     << keyframe.sonar_points->size() << ',' << version << ','
                     << graph_time_ms << '\n';
        diagnostics_.flush();
    }
    write_summary();
}

void LoopClosureManager::write_summary() const
{
    if (config_.diagnostics_directory.empty())
    {
        return;
    }
    const LoopClosureStats final_stats = stats();
    const PoseGraphSnapshot graph = graph_snapshot();
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    const auto output = config_.diagnostics_directory / "loop_closure_summary.json";
    const auto temporary = config_.diagnostics_directory /
                           "loop_closure_summary.json.tmp";
    std::ofstream summary(temporary);
    summary << "{\n"
            << "  \"submitted\": " << final_stats.submitted << ",\n"
            << "  \"processed\": " << final_stats.processed << ",\n"
            << "  \"failed\": " << final_stats.failed << ",\n"
            << "  \"dropped\": " << final_stats.dropped << ",\n"
            << "  \"graph_version\": " << final_stats.graph_version << ",\n"
            << "  \"graph_nodes\": " << graph.node_count << ",\n"
            << "  \"graph_factors\": " << graph.factor_count << ",\n"
            << "  \"graph_loop_factors\": " << graph.loop_factor_count << ",\n"
            << "  \"loops_accepted\": " << final_stats.loops_accepted << ",\n"
            << "  \"loops_rejected\": " << final_stats.loops_rejected << ",\n"
            << "  \"shadow_builds_started\": " << final_stats.shadow_builds_started << ",\n"
            << "  \"shadow_builds_ready\": " << final_stats.shadow_builds_ready << ",\n"
            << "  \"shadow_builds_failed\": " << final_stats.shadow_builds_failed << ",\n"
            << "  \"shadow_builds_stale\": " << final_stats.shadow_builds_stale << ",\n"
            << "  \"shadow_build_time_ms_sum\": " << final_stats.shadow_build_time_ms_sum << ",\n"
            << "  \"shadow_build_time_ms_max\": " << final_stats.shadow_build_time_ms_max << ",\n"
            << "  \"registrations_ready\": " << final_stats.registrations_ready << ",\n"
            << "  \"registrations_rejected\": " << final_stats.registrations_rejected << ",\n"
            << "  \"registration_time_ms_sum\": " << final_stats.registration_time_ms_sum << ",\n"
            << "  \"registration_time_ms_max\": " << final_stats.registration_time_ms_max << ",\n"
            << "  \"corrections_committed\": " << final_stats.corrections_committed << ",\n"
            << "  \"corrections_rejected\": " << final_stats.corrections_rejected << ",\n"
            << "  \"graph_time_ms_sum\": " << final_stats.graph_time_ms_sum
            << ",\n"
            << "  \"graph_time_ms_max\": " << final_stats.graph_time_ms_max
            << "\n}\n";
    summary.close();
    std::error_code error;
    std::filesystem::rename(temporary, output, error);
    if (error)
    {
        std::filesystem::remove(output, error);
        error.clear();
        std::filesystem::rename(temporary, output, error);
    }
}

void LoopClosureManager::write_registration_diagnostic(
    const RegistrationResult &result)
{
    if (!registration_diagnostics_)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    registration_diagnostics_ << result.graph_version << ','
                              << result.shadow_tree_generation << ','
                              << result.scan_generation << ','
                              << std::setprecision(17) << result.scan_timestamp << ','
                              << (result.valid ? 1 : 0) << ','
                              << (result.converged ? 1 : 0) << ','
                              << std::quoted(result.reason) << ','
                              << result.initial_effective_points << ','
                              << result.final_effective_points << ','
                              << result.initial_residual_mean_m << ','
                              << result.initial_residual_p95_m << ','
                              << result.final_residual_mean_m << ','
                              << result.final_residual_p95_m << ','
                              << result.information_min_eigenvalue << ','
                              << result.information_condition << ','
                              << result.iterations << ','
                              << result.registration_time_ms << '\n';
    registration_diagnostics_.flush();
}

void LoopClosureManager::write_shadow_diagnostic(
    const ShadowMapResult &result,
    const std::string &status)
{
    if (!shadow_diagnostics_)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    shadow_diagnostics_ << result.graph_version << ','
                        << result.source_tree_generation << ',' << status << ','
                        << std::quoted(result.reason) << ','
                        << result.selected_keyframes << ',' << result.input_points << ','
                        << result.filtered_points << ',' << std::setprecision(17)
                        << result.reconstruction_time_ms << ','
                        << result.downsample_time_ms << ','
                        << result.tree_build_time_ms << ','
                        << result.estimated_tree_bytes << '\n';
    shadow_diagnostics_.flush();
}

void LoopClosureManager::write_loop_diagnostic(
    const LoopConstraint &constraint,
    const LoopEvaluation &evaluation)
{
    if (!loop_diagnostics_)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    loop_diagnostics_ << constraint.from_id << ',' << constraint.to_id << ','
                      << (evaluation.accepted ? 1 : 0) << ','
                      << std::quoted(evaluation.reason) << ','
                      << evaluation.graph_version << ',' << std::setprecision(17)
                      << evaluation.graph_error_before << ','
                      << evaluation.graph_error_after << ','
                      << evaluation.loop_translation_error_before << ','
                      << evaluation.loop_translation_error_after << ','
                      << evaluation.loop_rotation_error_before_rad << ','
                      << evaluation.loop_rotation_error_after_rad << ','
                      << evaluation.optimization_time_ms << '\n';
    loop_diagnostics_.flush();
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
    result.shadow_build_time_ms_sum = shadow_build_time_ms_sum_.load();
    result.shadow_build_time_ms_max = shadow_build_time_ms_max_.load();
    result.registrations_ready = registrations_ready_.load();
    result.registrations_rejected = registrations_rejected_.load();
    result.registration_time_ms_sum = registration_time_ms_sum_.load();
    result.registration_time_ms_max = registration_time_ms_max_.load();
    result.corrections_committed = corrections_committed_.load();
    result.corrections_rejected = corrections_rejected_.load();
    result.graph_time_ms_sum = graph_time_ms_sum_.load();
    result.graph_time_ms_max = graph_time_ms_max_.load();
    return result;
}

std::shared_ptr<const ShadowMapResult> LoopClosureManager::shadow_map_snapshot() const
{
    std::lock_guard<std::mutex> lock(shadow_result_mutex_);
    return latest_shadow_map_;
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

void LoopClosureManager::notify_correction_result(bool committed)
{
    if (committed)
    {
        ++corrections_committed_;
    }
    else
    {
        ++corrections_rejected_;
    }
    write_summary();
}

PoseGraphSnapshot LoopClosureManager::graph_snapshot() const
{
    return pose_graph_.snapshot();
}

}  // namespace uwfl2::loop_closure

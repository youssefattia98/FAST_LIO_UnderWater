#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "loop_closure/bounded_queue.hpp"
#include "loop_closure/loop_closure_manager.hpp"
#include "loop_closure/loop_closure_types.hpp"
#include "loop_closure/pose_graph.hpp"
#include "loop_closure/shadow_map.hpp"
#include "loop_closure/state_transport.hpp"

namespace lc = uwfl2::loop_closure;

namespace
{

lc::Pose3d pose(const Eigen::Vector3d &translation, const Eigen::Vector3d &axis,
                double angle)
{
    return {Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis.normalized())), translation};
}

void expect_pose_near(const lc::Pose3d &actual, const lc::Pose3d &expected,
                      double tolerance = 1e-8)
{
    EXPECT_NEAR((actual.translation - expected.translation).norm(), 0.0, tolerance);
    EXPECT_NEAR(actual.rotation.angularDistance(expected.rotation), 0.0, tolerance);
}

lc::Keyframe keyframe(std::uint64_t id, const lc::Pose3d &T_local_vehicle)
{
    lc::Keyframe result;
    result.id = id;
    result.timestamp = static_cast<double>(id);
    result.T_local_vehicle = T_local_vehicle;
    result.pose_covariance = lc::Matrix6d::Identity() * 1e-3;
    result.T_vehicle_sonar = pose({0.1, -0.02, 0.2}, Eigen::Vector3d::UnitY(), 0.2);
    result.sonar_points = std::make_shared<const std::vector<lc::PointXYZI>>(
        30, lc::PointXYZI{});
    return result;
}

}  // namespace

TEST(LoopClosurePose, UsesDocumentedFullSe3Direction)
{
    const lc::Pose3d T_local_a =
        pose({1.0, -2.0, 0.7}, Eigen::Vector3d(1.0, 2.0, -1.0), 0.35);
    const lc::Pose3d T_a_b =
        pose({-0.4, 0.3, 1.2}, Eigen::Vector3d(-1.0, 0.5, 2.0), -0.42);
    const lc::Pose3d T_local_b = lc::compose(T_local_a, T_a_b);

    expect_pose_near(lc::between(T_local_a, T_local_b), T_a_b);
    expect_pose_near(lc::compose(T_local_a, lc::between(T_local_a, T_local_b)),
                     T_local_b);
}

TEST(LoopClosurePose, GtsamConversionPreservesRollPitchYawAndZ)
{
    const lc::Pose3d input =
        pose({4.0, -3.0, 2.0}, Eigen::Vector3d(0.3, -0.8, 0.5), 1.1);
    expect_pose_near(lc::FullSe3PoseGraph::from_gtsam(
                         lc::FullSe3PoseGraph::to_gtsam(input)),
                     input);
}

TEST(LoopClosureCovariance, ReordersIkfPositionAttitudeToGraphAttitudePosition)
{
    Eigen::Matrix<double, 27, 27> covariance =
        Eigen::Matrix<double, 27, 27>::Zero();
    covariance.block<3, 3>(0, 0).diagonal() << 1.0, 2.0, 3.0;
    covariance.block<3, 3>(3, 3).diagonal() << 4.0, 5.0, 6.0;
    covariance.block<3, 3>(0, 3).setConstant(0.25);
    covariance.block<3, 3>(3, 0) = covariance.block<3, 3>(0, 3).transpose();

    const lc::Matrix6d graph = lc::extract_graph_pose_covariance(covariance);
    EXPECT_DOUBLE_EQ(graph(0, 0), 4.0);
    EXPECT_DOUBLE_EQ(graph(2, 2), 6.0);
    EXPECT_DOUBLE_EQ(graph(3, 3), 1.0);
    EXPECT_DOUBLE_EQ(graph(5, 5), 3.0);
    EXPECT_DOUBLE_EQ(graph(0, 3), 0.25);
}

TEST(KeyframeSelector, SelectsVerticalAndRollMotion)
{
    lc::KeyframeSelectionConfig config;
    config.translation_m = 0.5;
    config.rotation_rad = 0.2;
    config.minimum_interval_s = 0.1;
    config.maximum_interval_s = 10.0;
    config.minimum_points = 5;
    lc::KeyframeSelector selector(config);
    const lc::Pose3d initial;
    ASSERT_TRUE(selector.should_select(1.0, initial, 5));
    selector.accept(1.0, initial);

    EXPECT_TRUE(selector.should_select(
        1.2, pose({0.0, 0.0, 0.6}, Eigen::Vector3d::UnitX(), 0.0), 5));
    EXPECT_TRUE(selector.should_select(
        1.2, pose(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 0.3), 5));
    EXPECT_FALSE(selector.should_select(
        1.05, pose({1.0, 0.0, 0.0}, Eigen::Vector3d::UnitZ(), 0.5), 5));
}

TEST(BoundedQueue, DropsOldestWithoutBlockingProducer)
{
    lc::BoundedQueue<int> queue(2);
    ASSERT_TRUE(queue.push_latest(1));
    ASSERT_TRUE(queue.push_latest(2));
    ASSERT_TRUE(queue.push_latest(3));
    EXPECT_EQ(queue.dropped(), 1U);
    EXPECT_EQ(queue.wait_pop(), 2);
    EXPECT_EQ(queue.wait_pop(), 3);
    queue.close();
    EXPECT_FALSE(queue.wait_pop().has_value());
}

TEST(FullSe3PoseGraph, AddsPriorAndConsecutiveOdometryFactors)
{
    lc::FullSe3PoseGraph graph;
    const lc::Pose3d first =
        pose({0.0, 0.0, 0.0}, Eigen::Vector3d::UnitX(), 0.15);
    const lc::Pose3d second =
        pose({1.0, 0.2, -0.4}, Eigen::Vector3d::UnitY(), -0.25);
    const lc::Pose3d third =
        pose({2.1, -0.1, -0.8}, Eigen::Vector3d::UnitZ(), 0.45);
    EXPECT_EQ(graph.append_keyframe(keyframe(0, first)), 1U);
    EXPECT_EQ(graph.append_keyframe(keyframe(1, second)), 2U);
    EXPECT_EQ(graph.append_keyframe(keyframe(2, third)), 3U);

    const lc::PoseGraphSnapshot snapshot = graph.snapshot();
    ASSERT_EQ(snapshot.node_count, 3U);
    EXPECT_EQ(snapshot.factor_count, 3U);
    ASSERT_EQ(snapshot.optimized_poses.size(), 3U);
    expect_pose_near(snapshot.optimized_poses[0], first, 1e-6);
    expect_pose_near(snapshot.optimized_poses[1], second, 1e-6);
    expect_pose_near(snapshot.optimized_poses[2], third, 1e-6);
}

TEST(FullSe3PoseGraph, TransactionalLoopReducesFullSe3Residual)
{
    lc::PoseGraphConfig config;
    config.loop_maximum_initial_nis = 1e6;
    lc::FullSe3PoseGraph graph(config);
    for (std::uint64_t id = 0; id < 8; ++id)
    {
        const double fraction = static_cast<double>(id) / 7.0;
        const lc::Pose3d drifted = pose(
            {static_cast<double>(id) * 1.1, 0.2 * fraction, 0.7 * fraction},
            Eigen::Vector3d(1.0, 0.4, 0.3), 0.18 * fraction);
        ASSERT_EQ(graph.append_keyframe(keyframe(id, drifted)), id + 1);
    }

    lc::LoopConstraint loop;
    loop.from_id = 0;
    loop.to_id = 7;
    loop.T_from_to = pose({7.0, 0.0, 0.0}, Eigen::Vector3d::UnitX(), 0.0);
    loop.covariance = lc::Matrix6d::Identity() * 0.01;
    const lc::LoopEvaluation evaluation = graph.try_add_loop(loop);

    ASSERT_TRUE(evaluation.accepted) << evaluation.reason;
    EXPECT_LT(evaluation.graph_error_after, evaluation.graph_error_before);
    EXPECT_LT(evaluation.loop_translation_error_after,
              evaluation.loop_translation_error_before);
    EXPECT_LT(evaluation.loop_rotation_error_after_rad,
              evaluation.loop_rotation_error_before_rad);
    const auto snapshot = graph.snapshot();
    EXPECT_EQ(snapshot.loop_factor_count, 1U);
    EXPECT_EQ(snapshot.factor_count, 9U);
    EXPECT_LT(snapshot.optimized_poses.back().translation.z(),
              snapshot.raw_poses.back().translation.z());
    EXPECT_LT(lc::rotation_distance_rad(snapshot.optimized_poses.back(),
                                        lc::Pose3d{}),
              lc::rotation_distance_rad(snapshot.raw_poses.back(), lc::Pose3d{}));
}

TEST(FullSe3PoseGraph, RejectsStatisticallyInconsistentLoop)
{
    lc::FullSe3PoseGraph graph;
    for (std::uint64_t id = 0; id < 8; ++id)
    {
        ASSERT_EQ(graph.append_keyframe(keyframe(
                      id, pose({static_cast<double>(id), 0.0, 0.0},
                               Eigen::Vector3d::UnitZ(), 0.0))),
                  id + 1);
    }
    lc::LoopConstraint loop;
    loop.from_id = 0;
    loop.to_id = 7;
    loop.T_from_to = pose({6.4, 0.0, 0.0}, Eigen::Vector3d::UnitZ(), 0.0);
    loop.covariance = lc::Matrix6d::Identity() * 1e-6;
    EXPECT_EQ(graph.try_add_loop(loop).reason, "initial_loop_nis_too_large");
    EXPECT_EQ(graph.snapshot().loop_factor_count, 0U);
}

TEST(FullSe3PoseGraph, NisIncludesAccumulatedRelativePoseUncertainty)
{
    lc::FullSe3PoseGraph graph;
    constexpr std::uint64_t final_id = 39;
    for (std::uint64_t id = 0; id <= final_id; ++id)
    {
        ASSERT_EQ(graph.append_keyframe(keyframe(
                      id, pose({1.02 * static_cast<double>(id), 0.0, 0.0},
                               Eigen::Vector3d::UnitZ(), 0.0))),
                  id + 1);
    }

    lc::LoopConstraint loop;
    loop.from_id = 0;
    loop.to_id = final_id;
    loop.T_from_to = pose({static_cast<double>(final_id), 0.0, 0.0},
                          Eigen::Vector3d::UnitZ(), 0.0);
    loop.covariance = lc::Matrix6d::Identity() * 1e-3;
    const auto evaluation = graph.try_add_loop(loop);

    ASSERT_TRUE(evaluation.accepted) << evaluation.reason;
    EXPECT_LT(evaluation.initial_nis, 12.592);
    EXPECT_LT(evaluation.loop_translation_error_after,
              evaluation.loop_translation_error_before);
}

TEST(FullSe3PoseGraph, RejectedLoopLeavesCommittedGraphUnchanged)
{
    lc::FullSe3PoseGraph graph;
    for (std::uint64_t id = 0; id < 8; ++id)
    {
        ASSERT_EQ(graph.append_keyframe(keyframe(
                      id, pose({static_cast<double>(id), 0.0, 0.0},
                               Eigen::Vector3d::UnitZ(), 0.0))),
                  id + 1);
    }
    const auto before = graph.snapshot();
    lc::LoopConstraint reversed;
    reversed.from_id = 0;
    reversed.to_id = 7;
    reversed.T_from_to = pose({-7.0, 0.0, 0.0}, Eigen::Vector3d::UnitZ(), 0.0);
    reversed.covariance = lc::Matrix6d::Identity() * 0.01;
    const auto evaluation = graph.try_add_loop(reversed);
    ASSERT_FALSE(evaluation.accepted);
    EXPECT_EQ(evaluation.reason, "initial_loop_residual_too_large");
    const auto after = graph.snapshot();
    EXPECT_EQ(after.version, before.version);
    EXPECT_EQ(after.factor_count, before.factor_count);
    EXPECT_EQ(after.loop_factor_count, 0U);
    for (std::size_t index = 0; index < before.optimized_poses.size(); ++index)
    {
        expect_pose_near(after.optimized_poses[index],
                         before.optimized_poses[index]);
    }
}

TEST(FullSe3PoseGraph, RejectsStaleAndNonFiniteConstraints)
{
    lc::FullSe3PoseGraph graph;
    ASSERT_EQ(graph.append_keyframe(keyframe(0, lc::Pose3d{})), 1U);
    lc::LoopConstraint stale;
    stale.from_id = 0;
    stale.to_id = 99;
    EXPECT_EQ(graph.try_add_loop(stale).reason, "unknown_keyframe");

    lc::LoopConstraint invalid;
    invalid.from_id = 0;
    invalid.to_id = 1;
    invalid.T_from_to.translation.x() =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(graph.try_add_loop(invalid).reason,
              "non_finite_or_invalid_covariance");
}

TEST(FullSe3PoseGraph, RejectsImpossibleConstraintAfterTemporaryOptimization)
{
    lc::FullSe3PoseGraph graph;
    for (std::uint64_t id = 0; id < 8; ++id)
    {
        ASSERT_EQ(graph.append_keyframe(keyframe(
                      id, pose({static_cast<double>(id), 0.0, 0.0},
                               Eigen::Vector3d::UnitZ(), 0.0))),
                  id + 1);
    }
    const auto before = graph.snapshot();
    lc::LoopConstraint impossible;
    impossible.from_id = 0;
    impossible.to_id = 7;
    impossible.T_from_to = pose({100.0, 50.0, -40.0},
                                Eigen::Vector3d(1.0, 1.0, 1.0), 2.5);
    impossible.covariance = lc::Matrix6d::Identity() * 1e-4;
    impossible.test_override = true;
    const auto evaluation = graph.try_add_loop(impossible);
    EXPECT_FALSE(evaluation.accepted);
    const auto after = graph.snapshot();
    EXPECT_EQ(after.version, before.version);
    EXPECT_EQ(after.factor_count, before.factor_count);
    EXPECT_EQ(after.loop_factor_count, 0U);
}

TEST(LoopClosureManager, DrainsWorkerAndShutsDownCleanly)
{
    lc::LoopClosureConfig config;
    config.enabled = true;
    config.queue_capacity = 4;
    config.keyframes.minimum_points = 3;
    config.keyframes.minimum_interval_s = 0.0;
    config.keyframes.translation_m = 0.1;
    std::vector<lc::PointXYZI> points(3);

    lc::LoopClosureManager manager(config);
    ASSERT_TRUE(manager.try_submit(1.0, lc::Pose3d{}, lc::Matrix6d::Identity(),
                                   lc::Pose3d{}, points, 0));
    for (int attempt = 0; attempt < 100 && manager.stats().processed == 0; ++attempt)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    EXPECT_EQ(manager.stats().processed, 1U);
    EXPECT_EQ(manager.graph_snapshot().node_count, 1U);
}

TEST(LoopClosureManager, PreselectsAndSharesLatestScanSnapshot)
{
    lc::LoopClosureConfig config;
    config.enabled = true;
    config.queue_capacity = 4;
    config.keyframes.minimum_points = 3;
    config.keyframes.minimum_interval_s = 0.0;
    config.keyframes.translation_m = 1.0;
    config.keyframes.rotation_rad = 1.0;
    config.keyframes.maximum_interval_s = 5.0;
    std::vector<lc::PointXYZI> points(3);
    points[0].x = 1.0F;

    lc::LoopClosureManager manager(config);
    const auto snapshot = manager.notify_latest_scan(
        1.0, 1, 0, lc::Pose3d{}, lc::Pose3d{}, points);
    ASSERT_TRUE(snapshot);
    ASSERT_TRUE(manager.should_select_keyframe(1.0, lc::Pose3d{},
                                               snapshot->size()));
    const std::vector<lc::PointXYZI> map_points;
    ASSERT_TRUE(manager.try_submit_snapshot(
        1.0, lc::Pose3d{}, lc::Matrix6d::Identity(), lc::Pose3d{}, snapshot,
        map_points, 0));
    for (int attempt = 0; attempt < 100 && manager.stats().processed == 0;
         ++attempt)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    const auto graph = manager.graph_snapshot();
    ASSERT_EQ(graph.node_count, 1U);
    ASSERT_EQ(graph.keyframes.size(), 1U);
    EXPECT_EQ(graph.keyframes.front().sonar_points.get(), snapshot.get());
    EXPECT_FALSE(manager.should_select_keyframe(1.1, lc::Pose3d{},
                                                snapshot->size()));
}

namespace
{

lc::PoseGraphSnapshot shadow_graph(const lc::Pose3d &optimized_pose)
{
    lc::PoseGraphSnapshot graph;
    graph.version = 7;
    graph.ids = {0};
    graph.timestamps = {1.0};
    graph.raw_poses = {lc::Pose3d{}};
    graph.optimized_poses = {optimized_pose};
    lc::Keyframe frame = keyframe(0, lc::Pose3d{});
    frame.T_vehicle_sonar = pose({0.2, -0.1, 0.3},
                                 Eigen::Vector3d::UnitY(), 0.25);
    frame.tree_generation = 4;
    auto points = std::make_shared<std::vector<lc::PointXYZI>>();
    points->push_back({0.0F, 0.0F, 0.0F, 1.0F});
    points->push_back({1.0F, 0.0F, 0.0F, 2.0F});
    points->push_back({0.0F, 1.0F, 0.5F, 3.0F});
    frame.sonar_points = points;
    graph.keyframes = {frame};
    graph.node_count = 1;
    return graph;
}

}  // namespace

TEST(ShadowMap, ReconstructsKnownFullSe3Transform)
{
    const lc::Pose3d corrected = pose({2.0, -1.0, 3.0},
                                      Eigen::Vector3d(1.0, 2.0, -0.5), 0.6);
    lc::ShadowMapRequest request{shadow_graph(corrected), 4};
    lc::ShadowMapConfig config;
    config.radius_m = 10.0;
    config.voxel_size_m = 0.01;
    const auto points = lc::ShadowMapBuilder::reconstruct_and_downsample(
        request, config);
    ASSERT_EQ(points.size(), 3U);
    const lc::Pose3d T_local_sonar = lc::compose(
        corrected, request.graph.keyframes[0].T_vehicle_sonar);
    std::vector<Eigen::Vector3d> expected;
    for (const auto &point : *request.graph.keyframes[0].sonar_points)
    {
        expected.push_back(T_local_sonar.rotation *
                               Eigen::Vector3d(point.x, point.y, point.z) +
                           T_local_sonar.translation);
    }
    for (const auto &point : points)
    {
        const Eigen::Vector3d actual(point.x, point.y, point.z);
        const auto nearest = std::min_element(
            expected.begin(), expected.end(), [&](const auto &lhs, const auto &rhs) {
                return (lhs - actual).squaredNorm() < (rhs - actual).squaredNorm();
            });
        ASSERT_NE(nearest, expected.end());
        EXPECT_LT((*nearest - actual).norm(), 1e-5);
    }
}

TEST(ShadowMap, KeepsFullCorrectedHistoryOutsideTheLiveMapRadius)
{
    lc::PoseGraphSnapshot graph;
    graph.version = 9;
    graph.ids = {0, 1};
    graph.timestamps = {1.0, 2.0};
    graph.raw_poses = {
        pose({-20.0, 1.0, 0.5}, Eigen::Vector3d::UnitX(), 0.1),
        pose({20.0, -1.0, -0.5}, Eigen::Vector3d::UnitY(), -0.1)};
    graph.optimized_poses = {
        pose({-19.0, 2.0, 1.5}, Eigen::Vector3d::UnitZ(), 0.2),
        graph.raw_poses[1]};
    for (std::size_t index = 0; index < 2; ++index)
    {
        lc::Keyframe frame = keyframe(index, graph.raw_poses[index]);
        frame.map_points_world =
            std::make_shared<const std::vector<lc::PointXYZI>>(
                1, lc::PointXYZI{static_cast<float>(graph.raw_poses[index].translation.x()),
                                 static_cast<float>(graph.raw_poses[index].translation.y()),
                                 static_cast<float>(graph.raw_poses[index].translation.z()),
                                 static_cast<float>(index)});
        graph.keyframes.push_back(std::move(frame));
    }
    graph.node_count = graph.keyframes.size();

    lc::ShadowMapConfig config;
    config.radius_m = 5.0;
    config.voxel_size_m = 0.01;
    const auto result = lc::ShadowMapBuilder(config).build({graph, 3});
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.selected_keyframes, 1U);
    ASSERT_TRUE(result.corrected_history_points);
    ASSERT_EQ(result.corrected_history_points->size(), 2U);

    lc::ShadowMapConfig history_config = config;
    history_config.radius_m = 0.0;
    history_config.maximum_keyframes = graph.keyframes.size();
    const lc::ShadowMapRequest request{graph, 3};
    const auto expected_history =
        lc::ShadowMapBuilder::reconstruct_and_downsample(
            request, history_config);
    ASSERT_EQ(result.corrected_history_points->size(), expected_history.size());
    for (std::size_t index = 0; index < expected_history.size(); ++index)
    {
        EXPECT_FLOAT_EQ((*result.corrected_history_points)[index].x,
                        expected_history[index].x);
        EXPECT_FLOAT_EQ((*result.corrected_history_points)[index].y,
                        expected_history[index].y);
        EXPECT_FLOAT_EQ((*result.corrected_history_points)[index].z,
                        expected_history[index].z);
    }
    const auto expected_active =
        lc::ShadowMapBuilder::reconstruct_and_downsample(request, config);
    EXPECT_EQ(result.filtered_points, expected_active.size());
    for (const auto &expected_point : expected_active)
    {
        lc::ShadowPointVector nearest;
        std::vector<float> squared_distances;
        result.tree->Nearest_Search(expected_point, 1, nearest,
                                    squared_distances);
        ASSERT_EQ(nearest.size(), 1U);
        EXPECT_LT(squared_distances.front(), 1e-10F);
    }

    const lc::Pose3d first_correction = lc::compose(
        graph.optimized_poses[0], lc::inverse(graph.raw_poses[0]));
    const auto &first_raw = graph.keyframes[0].map_points_world->front();
    const Eigen::Vector3d expected =
        first_correction.rotation *
            Eigen::Vector3d(first_raw.x, first_raw.y, first_raw.z) +
        first_correction.translation;
    const auto &actual = result.corrected_history_points->front();
    EXPECT_NEAR(actual.x, expected.x(), 1e-5);
    EXPECT_NEAR(actual.y, expected.y(), 1e-5);
    EXPECT_NEAR(actual.z, expected.z(), 1e-5);
}

TEST(ShadowMap, GloballyCompactsOverlappingHistoricalSubmaps)
{
    lc::PoseGraphSnapshot graph;
    graph.version = 3;
    graph.ids = {0, 1};
    graph.timestamps = {1.0, 2.0};
    graph.raw_poses = {
        pose({-20.0, 0.0, 0.0}, Eigen::Vector3d::UnitZ(), 0.0),
        pose({20.0, 0.0, 0.0}, Eigen::Vector3d::UnitZ(), 0.0)};
    graph.optimized_poses = graph.raw_poses;
    for (std::size_t index = 0; index < 2; ++index)
    {
        lc::Keyframe frame = keyframe(index, graph.raw_poses[index]);
        frame.map_points_world =
            std::make_shared<const std::vector<lc::PointXYZI>>(
                1, lc::PointXYZI{0.01F + 0.01F * static_cast<float>(index),
                                 0.0F, 0.0F, static_cast<float>(index)});
        graph.keyframes.push_back(std::move(frame));
    }
    graph.node_count = graph.keyframes.size();

    lc::ShadowMapConfig config;
    config.radius_m = 5.0;
    config.voxel_size_m = 0.2;
    const auto result = lc::ShadowMapBuilder(config).build({graph, 2});
    ASSERT_TRUE(result.valid) << result.reason;
    ASSERT_TRUE(result.corrected_history_points);
    EXPECT_EQ(result.selected_keyframes, 1U);
    EXPECT_EQ(result.corrected_history_points->size(), 1U);
}

TEST(ShadowMap, VoxelSelectionIsDeterministicAndMatchesCenterRule)
{
    auto graph = shadow_graph(lc::Pose3d{});
    graph.keyframes[0].T_vehicle_sonar = lc::Pose3d{};
    auto points = std::make_shared<std::vector<lc::PointXYZI>>();
    points->push_back({0.11F, 0.11F, 0.11F, 1.0F});
    points->push_back({0.49F, 0.49F, 0.49F, 2.0F});
    points->push_back({1.1F, 0.0F, 0.0F, 3.0F});
    graph.keyframes[0].sonar_points = points;
    lc::ShadowMapRequest request{graph, 4};
    lc::ShadowMapConfig config;
    config.radius_m = 10.0;
    config.voxel_size_m = 1.0;
    const auto first = lc::ShadowMapBuilder::reconstruct_and_downsample(request, config);
    std::reverse(points->begin(), points->end());
    const auto second = lc::ShadowMapBuilder::reconstruct_and_downsample(request, config);
    ASSERT_EQ(first.size(), 2U);
    ASSERT_EQ(second.size(), first.size());
    for (std::size_t index = 0; index < first.size(); ++index)
    {
        EXPECT_FLOAT_EQ(first[index].x, second[index].x);
        EXPECT_FLOAT_EQ(first[index].y, second[index].y);
        EXPECT_FLOAT_EQ(first[index].z, second[index].z);
    }
    EXPECT_FLOAT_EQ(first[0].x, 0.49F);
}

TEST(ShadowMap, BuildsQueryableIkdTreeAndRejectsStaleGeneration)
{
    lc::ShadowMapRequest request{shadow_graph(lc::Pose3d{}), 4};
    lc::ShadowMapConfig config;
    config.radius_m = 10.0;
    config.voxel_size_m = 0.01;
    const auto result = lc::ShadowMapBuilder(config).build(request);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.filtered_points, 3U);
    lc::ShadowPoint query;
    const lc::Pose3d T_local_sonar = request.graph.keyframes[0].T_vehicle_sonar;
    query.x = static_cast<float>(T_local_sonar.translation.x());
    query.y = static_cast<float>(T_local_sonar.translation.y());
    query.z = static_cast<float>(T_local_sonar.translation.z());
    lc::ShadowPointVector nearest;
    std::vector<float> squared_distances;
    result.tree->Nearest_Search(query, 1, nearest, squared_distances);
    ASSERT_EQ(nearest.size(), 1U);
    EXPECT_LT(squared_distances[0], 1e-10F);
    EXPECT_TRUE(lc::shadow_result_matches_graph_version(
        result, request.graph.version));
    EXPECT_FALSE(lc::shadow_result_matches_graph_version(
        result, request.graph.version + 1));
    auto appended_graph = request.graph;
    ++appended_graph.version;
    EXPECT_TRUE(lc::shadow_result_matches_graph(result, appended_graph));
    ++appended_graph.loop_factor_count;
    EXPECT_FALSE(lc::shadow_result_matches_graph(result, appended_graph));
}

TEST(ShadowMap, EnforcesInputPointBudget)
{
    lc::ShadowMapRequest request{shadow_graph(lc::Pose3d{}), 4};
    lc::ShadowMapConfig config;
    config.radius_m = 10.0;
    config.voxel_size_m = 0.1;
    config.maximum_input_points = 2;
    const auto result = lc::ShadowMapBuilder(config).build(request);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.reason, "Shadow-map input point budget exceeded");
    EXPECT_FALSE(result.tree);
}

TEST(LatestScanRegistration, RecoversIndependentAndCombinedFullSe3Errors)
{
    lc::ShadowPointVector map_points;
    auto add_point = [&](double x, double y, double z) {
        lc::ShadowPoint point;
        point.x = static_cast<float>(x);
        point.y = static_cast<float>(y);
        point.z = static_cast<float>(z);
        map_points.push_back(point);
    };
    for (double first = -1.5; first <= 1.5; first += 0.15)
    {
        for (double second = -1.5; second <= 1.5; second += 0.15)
        {
            add_point(-2.0, first, second);
            add_point(2.0, first, second);
            add_point(first, -2.0, second);
            add_point(first, 2.0, second);
            add_point(first, second, -2.0);
            add_point(first, second, 2.0);
        }
    }
    lc::ShadowMapResult shadow;
    shadow.valid = true;
    shadow.graph_version = 1;
    shadow.graph_loop_factor_count = 0;
    shadow.shadow_tree_generation = 1;
    shadow.tree = std::make_shared<lc::ShadowTree>();
    shadow.tree->Build(map_points);

    auto scan_points = std::make_shared<std::vector<lc::PointXYZI>>();
    scan_points->reserve(map_points.size());
    for (const auto &point : map_points)
    {
        scan_points->push_back({point.x, point.y, point.z, 0.0F});
    }
    lc::PoseGraphSnapshot graph;
    graph.version = 1;
    graph.loop_factor_count = 0;
    graph.raw_poses = {lc::Pose3d{}};
    graph.optimized_poses = {lc::Pose3d{}};
    lc::RegistrationConfig config;
    config.minimum_effective_points = 100;
    config.maximum_neighbor_distance_m = 1.0;
    config.plane_fit_threshold_m = 0.03;
    config.maximum_registration_translation_m = 1.0;
    config.maximum_registration_rotation_rad = 0.5;
    lc::LatestScanRegistrar registrar(config);

    const std::vector<lc::Pose3d> perturbations = {
        pose({0.12, 0.0, 0.0}, Eigen::Vector3d::UnitX(), 0.0),
        pose({0.0, -0.12, 0.0}, Eigen::Vector3d::UnitY(), 0.0),
        pose({0.0, 0.0, 0.12}, Eigen::Vector3d::UnitZ(), 0.0),
        pose(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 0.05),
        pose(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY(), -0.05),
        pose(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 0.05),
        pose({0.08, -0.06, 0.1}, Eigen::Vector3d(1.0, -0.5, 0.8), 0.06),
    };
    for (std::size_t index = 0; index < perturbations.size(); ++index)
    {
        lc::LatestScanSnapshot scan;
        scan.timestamp = 1.0;
        scan.scan_generation = index + 1;
        scan.T_local_vehicle_raw = perturbations[index];
        scan.sonar_points = scan_points;
        const lc::RegistrationResult result =
            registrar.register_scan(scan, graph, shadow);
        ASSERT_TRUE(result.valid) << index << ": " << result.reason;
        EXPECT_LT(result.T_local_vehicle_registered.translation.norm(), 2e-3);
        EXPECT_LT(result.T_local_vehicle_registered.rotation.angularDistance(
                      Eigen::Quaterniond::Identity()),
                  2e-3);
        EXPECT_LE(result.final_residual_mean_m,
                  result.initial_residual_mean_m + 1e-9);
        EXPECT_TRUE(lc::covariance_is_valid(result.covariance));
    }
}

TEST(StateTransport, AnalyticJacobianMatchesFullSe3FiniteDifference)
{
    const Eigen::Matrix3d correction =
        Eigen::AngleAxisd(0.47, Eigen::Vector3d(1.0, -0.4, 0.7).normalized())
            .toRotationMatrix();
    const Eigen::Matrix3d base_rotation =
        Eigen::AngleAxisd(-0.31, Eigen::Vector3d(0.2, 1.0, -0.3).normalized())
            .toRotationMatrix();
    const auto analytic = lc::correction_transport_jacobian(correction);
    lc::Matrix27d numerical = lc::Matrix27d::Zero();
    const double epsilon = 1e-7;
    for (int column = 0; column < 27; ++column)
    {
        Eigen::Matrix<double, 27, 1> delta =
            Eigen::Matrix<double, 27, 1>::Zero();
        delta(column) = epsilon;
        Eigen::Matrix<double, 27, 1> output = delta;
        output.segment<3>(0) = correction * delta.segment<3>(0);
        const Eigen::Vector3d attitude_delta = delta.segment<3>(3);
        const Eigen::Matrix3d perturbed =
            base_rotation * Eigen::AngleAxisd(
                                attitude_delta.norm(),
                                attitude_delta.norm() > 0.0
                                    ? attitude_delta.normalized()
                                    : Eigen::Vector3d::UnitX())
                                .toRotationMatrix();
        const Eigen::Matrix3d relative =
            (correction * base_rotation).transpose() * correction * perturbed;
        output.segment<3>(3) =
            Eigen::AngleAxisd(relative).axis() * Eigen::AngleAxisd(relative).angle();
        output.segment<3>(12) = correction * delta.segment<3>(12);
        numerical.col(column) = output / epsilon;
    }
    EXPECT_LT((analytic - numerical).cwiseAbs().maxCoeff(), 2e-8);
}

TEST(StateTransport, KeepsProtectedBlocksAndProducesPsdCovariance)
{
    lc::Matrix27d covariance = lc::Matrix27d::Identity() * 0.01;
    covariance.block<3, 3>(0, 15).setConstant(1e-4);
    covariance.block<3, 3>(15, 0) = covariance.block<3, 3>(0, 15).transpose();
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
    lc::Matrix6d correction_covariance = lc::Matrix6d::Identity() * 1e-3;
    const auto transported = lc::transport_uwfl2_covariance(
        covariance, rotation, correction_covariance);
    EXPECT_TRUE(lc::covariance27_is_valid(transported));
    EXPECT_TRUE((transported.block<12, 12>(15, 15).isApprox(
        covariance.block<12, 12>(15, 15), 1e-14)));
    EXPECT_TRUE((transported.block<3, 3>(0, 15).isApprox(
        rotation * covariance.block<3, 3>(0, 15), 1e-14)));
}

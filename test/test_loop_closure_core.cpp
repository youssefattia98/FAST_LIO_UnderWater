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
    lc::FullSe3PoseGraph graph;
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

#include <chrono>
#include <cmath>
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

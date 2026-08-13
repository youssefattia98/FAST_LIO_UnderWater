#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "loop_closure/loop_closure_manager.hpp"
#include "loop_closure/std_detector.hpp"

namespace lc = uwfl2::loop_closure;

namespace
{

lc::Pose3d pose(const Eigen::Vector3d &translation,
                const Eigen::Vector3d &axis, double angle)
{
    return {Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis.normalized())),
            translation};
}

std::vector<lc::PointXYZI> scene()
{
    const std::vector<Eigen::Vector3d> centers = {
        {-4.3, -2.7, -1.4}, {-1.5, 3.2, 2.1}, {2.3, -3.5, 1.7},
        {4.4, 1.6, -2.2},   {-3.1, 4.7, -3.3}, {3.6, 4.2, 3.8},
        {0.7, -0.9, 4.6},   {-4.8, 0.3, 4.1},  {1.9, 5.4, -0.5},
        {5.2, -1.8, 4.5},   {-0.4, -5.1, -3.7}, {4.8, 4.9, -4.2}};
    std::vector<lc::PointXYZI> points;
    for (std::size_t index = 0; index < centers.size(); ++index)
    {
        Eigen::Vector3d normal(0.3 + 0.11 * index, -0.8 + 0.07 * index,
                               0.5 + 0.03 * index);
        normal.normalize();
        const Eigen::Vector3d first = normal.unitOrthogonal();
        const Eigen::Vector3d second = normal.cross(first).normalized();
        for (int row = -2; row <= 2; ++row)
        {
            for (int column = -2; column <= 2; ++column)
            {
                const Eigen::Vector3d value =
                    centers[index] + 0.035 * row * first +
                    0.041 * column * second;
                points.push_back({static_cast<float>(value.x()),
                                  static_cast<float>(value.y()),
                                  static_cast<float>(value.z()),
                                  static_cast<float>(index)});
            }
        }
    }
    return points;
}

std::vector<lc::PointXYZI> transform_points(
    const std::vector<lc::PointXYZI> &target,
    const lc::Pose3d &T_target_source)
{
    std::vector<lc::PointXYZI> source;
    source.reserve(target.size());
    const lc::Pose3d inverse = lc::inverse(T_target_source);
    for (const auto &point : target)
    {
        const Eigen::Vector3d value =
            inverse.rotation * Eigen::Vector3d(point.x, point.y, point.z) +
            inverse.translation;
        source.push_back({static_cast<float>(value.x()),
                          static_cast<float>(value.y()),
                          static_cast<float>(value.z()), point.intensity});
    }
    return source;
}

lc::Keyframe frame(std::uint64_t id, const lc::Pose3d &pose,
                   const std::vector<lc::PointXYZI> &points)
{
    lc::Keyframe result;
    result.id = id;
    result.timestamp = static_cast<double>(id);
    result.T_local_vehicle = pose;
    result.pose_covariance = lc::Matrix6d::Identity() * 1e-3;
    result.T_vehicle_sonar = lc::Pose3d{};
    result.sonar_points =
        std::make_shared<const std::vector<lc::PointXYZI>>(points);
    return result;
}

lc::StdConfig detector_config()
{
    lc::StdConfig config;
    config.minimum_keyframe_separation = 2;
    config.voxel_size_m = 0.8;
    config.minimum_voxel_points = 4;
    config.maximum_keypoints = 64;
    config.maximum_triangles = 600;
    config.triangle_side_resolution_m = 0.25;
    config.triangle_relative_error = 0.12;
    config.binary_similarity_minimum = 0.1;
    config.minimum_triangle_matches = 3;
    config.minimum_ransac_inliers = 3;
    config.ransac_vertex_threshold_m = 0.2;
    config.geometric_overlap_distance_m = 0.2;
    config.geometric_overlap_minimum = 0.75;
    return config;
}

bool wait_processed(lc::LoopClosureManager &manager, std::uint64_t count)
{
    for (int attempt = 0; attempt < 500; ++attempt)
    {
        if (manager.stats().processed >= count)
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

}  // namespace

TEST(StableTriangleDetector, RecoversFullSe3RevisitAfterConfirmation)
{
    lc::StdConfig config = detector_config();
    config.required_consistent_detections = 2;
    lc::StableTriangleDetector detector(config);
    const auto historical = scene();
    const lc::Pose3d expected =
        pose({1.2, -0.8, 0.6}, Eigen::Vector3d(1.0, -0.4, 0.7), 0.45);
    auto current = transform_points(historical, expected);
    std::reverse(current.begin(), current.end());

    EXPECT_FALSE(detector.process(frame(0, lc::Pose3d{}, historical)).loop);
    const auto first = detector.process(frame(3, expected, current));
    EXPECT_TRUE(first.proposed) << first.reason;
    EXPECT_EQ(first.reason, "awaiting_confirmation");
    const auto second = detector.process(frame(4, expected, current));
    ASSERT_TRUE(second.loop.has_value()) << second.reason;
    EXPECT_EQ(second.loop->from_id, 0U);
    EXPECT_EQ(second.loop->to_id, 4U);
    EXPECT_LT((second.loop->T_from_to.translation - expected.translation).norm(),
              0.08);
    EXPECT_LT(second.loop->T_from_to.rotation.angularDistance(expected.rotation),
              0.03);
}

TEST(StableTriangleDetector, AcceptsStrongLongSpanRevisitWithoutSecondScan)
{
    lc::StdConfig config = detector_config();
    config.required_consistent_detections = 2;
    config.single_detection_overlap_minimum = 0.80;
    config.accepted_loop_cooldown_s = 30.0;
    lc::StableTriangleDetector detector(config);
    const auto historical = scene();
    const lc::Pose3d expected =
        pose({1.2, -0.8, 0.6}, Eigen::Vector3d(1.0, -0.4, 0.7), 0.45);
    auto current = transform_points(historical, expected);
    std::reverse(current.begin(), current.end());

    EXPECT_FALSE(detector.process(frame(0, lc::Pose3d{}, historical)).loop);
    const auto result = detector.process(frame(20, expected, current));
    ASSERT_TRUE(result.loop.has_value()) << result.reason;
    EXPECT_EQ(result.reason, "confirmed_high_confidence");
    EXPECT_EQ(result.loop->from_id, 0U);
    EXPECT_EQ(result.loop->to_id, 20U);
    detector.notify_loop_accepted(20, 0, 20.0);
    EXPECT_FALSE(detector.process(frame(21, expected, current)).loop);
}

TEST(StableTriangleDetector, EnforcesMinimumLoopDuration)
{
    lc::StdConfig config = detector_config();
    config.required_consistent_detections = 1;
    config.minimum_loop_duration_s = 10.0;
    lc::StableTriangleDetector detector(config);
    const auto historical = scene();
    const lc::Pose3d expected =
        pose({1.2, -0.8, 0.6}, Eigen::Vector3d(1.0, -0.4, 0.7), 0.45);
    const auto current = transform_points(historical, expected);

    EXPECT_FALSE(detector.process(frame(0, lc::Pose3d{}, historical)).loop);
    EXPECT_FALSE(detector.process(frame(3, expected, current)).loop);
    const auto result = detector.process(frame(20, expected, current));
    ASSERT_TRUE(result.loop.has_value()) << result.reason;
}

TEST(StableTriangleDetector, ConfirmationToleranceRemainsFullSe3)
{
    lc::StdConfig config = detector_config();
    config.required_consistent_detections = 2;
    config.single_detection_overlap_minimum = 0.0;
    config.confirmation_translation_m = 0.3;
    config.confirmation_rotation_rad = 0.1;
    lc::StableTriangleDetector detector(config);
    const auto historical = scene();
    const lc::Pose3d expected =
        pose({1.2, -0.8, 0.6}, Eigen::Vector3d(1.0, -0.4, 0.7), 0.45);
    const auto current = transform_points(historical, expected);

    EXPECT_FALSE(detector.process(frame(0, lc::Pose3d{}, historical)).loop);
    EXPECT_EQ(detector.process(frame(3, expected, current)).reason,
              "awaiting_confirmation");
    const auto result = detector.process(frame(4, expected, current));
    ASSERT_TRUE(result.loop.has_value()) << result.reason;
    EXPECT_EQ(result.loop->from_id, 0U);
    EXPECT_EQ(result.loop->to_id, 4U);
}

TEST(StableTriangleDetector, RejectsSparseAndLowOverlapCandidates)
{
    lc::StdConfig config = detector_config();
    config.required_consistent_detections = 1;
    lc::StableTriangleDetector detector(config);
    const auto historical = scene();
    detector.process(frame(0, lc::Pose3d{}, historical));

    std::vector<lc::PointXYZI> sparse(20);
    for (std::size_t index = 0; index < sparse.size(); ++index)
    {
        sparse[index] = {static_cast<float>(2.0 * index),
                         static_cast<float>(-1.3 * index),
                         static_cast<float>(0.9 * index), 0.0F};
    }
    const auto sparse_result = detector.process(frame(2, lc::Pose3d{}, sparse));
    EXPECT_FALSE(sparse_result.loop);
    EXPECT_EQ(sparse_result.reason, "descriptor_rejected");

    const lc::Pose3d transform =
        pose({0.7, -0.4, 0.5}, Eigen::Vector3d(0.5, 1.0, -0.3), 0.3);
    auto contaminated = transform_points(historical, transform);
    for (int index = 0; index < 1000; ++index)
    {
        contaminated.push_back({20.0F + 1.1F * index,
                                -30.0F - 0.9F * index,
                                15.0F + 0.7F * index, 0.0F});
    }
    const auto false_result =
        detector.process(frame(5, transform, contaminated));
    EXPECT_FALSE(false_result.loop);
    EXPECT_EQ(false_result.reason, "geometry_rejected");
}

TEST(LoopClosureManager, AutomaticStdUsesTransactionalGraphGate)
{
    lc::LoopClosureConfig config;
    config.enabled = true;
    config.automatic_detection_enabled = true;
    config.queue_capacity = 4;
    config.keyframes.minimum_points = 20;
    config.keyframes.minimum_interval_s = 0.0;
    config.keyframes.translation_m = 0.05;
    config.pose_graph.loop_minimum_keyframe_separation = 2;
    config.std_detection = detector_config();
    config.std_detection.required_consistent_detections = 1;
    const auto historical = scene();
    const lc::Pose3d expected =
        pose({1.0, -0.7, 0.5}, Eigen::Vector3d(0.8, -0.3, 0.6), 0.35);
    const auto current = transform_points(historical, expected);
    std::vector<lc::PointXYZI> sparse(30);
    for (std::size_t index = 0; index < sparse.size(); ++index)
    {
        sparse[index] = {static_cast<float>(2.1 * index),
                         static_cast<float>(1.7 * index),
                         static_cast<float>(-1.2 * index), 0.0F};
    }

    lc::LoopClosureManager manager(config);
    ASSERT_TRUE(manager.try_submit(0.0, lc::Pose3d{}, lc::Matrix6d::Identity(),
                                   lc::Pose3d{}, historical, 0));
    ASSERT_TRUE(wait_processed(manager, 1));
    ASSERT_TRUE(manager.try_submit(1.0,
                                   pose({0.5, 0.2, -0.1},
                                        Eigen::Vector3d::UnitX(), 0.1),
                                   lc::Matrix6d::Identity(), lc::Pose3d{}, sparse, 0));
    ASSERT_TRUE(wait_processed(manager, 2));
    ASSERT_TRUE(manager.try_submit(2.0, expected, lc::Matrix6d::Identity(),
                                   lc::Pose3d{}, current, 0));
    ASSERT_TRUE(wait_processed(manager, 3));
    for (int attempt = 0; attempt < 500 && manager.stats().std_accepted == 0;
         ++attempt)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const auto stats = manager.stats();
    EXPECT_EQ(stats.std_processed, 3U);
    EXPECT_EQ(stats.std_accepted, 1U);
    EXPECT_EQ(stats.std_graph_rejected, 0U);
    EXPECT_EQ(manager.graph_snapshot().loop_factor_count, 1U);
}

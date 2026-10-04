#include <gtest/gtest.h>

#include "lidar_scan_quality.hpp"
#include "imu_gap_policy.hpp"
#include "sensor_timestamp_policy.hpp"

namespace
{

struct FakeEkf
{
    int state = 0;
    int covariance = 0;

    const int &get_x() const { return state; }
    const int &get_P() const { return covariance; }
    void change_x(int &value) { state = value; }
    void change_P(int &value) { covariance = value; }
};

}  // namespace

TEST(LidarScanQuality, LegacyDefaultsAcceptBoundary)
{
    const uwfl2::LidarScanQualityPolicy policy;
    EXPECT_TRUE(policy.input_is_sufficient(5));
    EXPECT_FALSE(policy.map_initialization_is_sufficient(5));
    EXPECT_TRUE(policy.map_initialization_is_sufficient(6));
    EXPECT_TRUE(policy.features_are_sufficient(1));
}

TEST(SensorTimestampPolicy, ReplacesOnlyImpossibleFutureTimestamps)
{
    const auto future = uwfl2::validate_sensor_timestamp(110.0, 100.0, 0.5);
    EXPECT_TRUE(future.used_arrival_time);
    EXPECT_DOUBLE_EQ(future.timestamp, 100.0);

    const auto delayed = uwfl2::validate_sensor_timestamp(90.0, 100.0, 0.5);
    EXPECT_FALSE(delayed.used_arrival_time);
    EXPECT_DOUBLE_EQ(delayed.timestamp, 90.0);

    const auto disabled = uwfl2::validate_sensor_timestamp(110.0, 100.0, 0.0);
    EXPECT_FALSE(disabled.used_arrival_time);

    EXPECT_DOUBLE_EQ(uwfl2::usable_arrival_timestamp(123.0), 123.0);
}

TEST(LidarScanQuality, RejectsSparseInput)
{
    const uwfl2::LidarScanQualityPolicy policy(50, 20);
    const auto result = policy.evaluate_update(49, 30, true);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.reason, uwfl2::LidarScanRejectionReason::kSparseInput);
}

TEST(LidarScanQuality, RejectsInsufficientCorrespondences)
{
    const uwfl2::LidarScanQualityPolicy policy(50, 20);
    const auto result = policy.evaluate_update(100, 19, true);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.reason,
              uwfl2::LidarScanRejectionReason::kInsufficientCorrespondences);
}

TEST(LidarScanQuality, AcceptsCompleteUpdate)
{
    const uwfl2::LidarScanQualityPolicy policy(50, 20);
    const auto result = policy.evaluate_update(50, 20, true);
    EXPECT_TRUE(result.accepted);
    EXPECT_TRUE(uwfl2::should_insert_lidar_scan(result));
}

TEST(LidarScanQuality, RejectedUpdateRestoresStateAndCovariance)
{
    FakeEkf ekf{3, 7};
    uwfl2::LidarUpdateTransaction<FakeEkf> transaction(ekf);
    ekf.state = 30;
    ekf.covariance = 70;
    const uwfl2::LidarUpdateResult rejected;
    transaction.finish(rejected);
    EXPECT_EQ(ekf.state, 3);
    EXPECT_EQ(ekf.covariance, 7);
    EXPECT_FALSE(uwfl2::should_insert_lidar_scan(rejected));
}

TEST(LidarScanQuality, AcceptedUpdateKeepsCorrection)
{
    FakeEkf ekf{3, 7};
    uwfl2::LidarUpdateTransaction<FakeEkf> transaction(ekf);
    ekf.state = 30;
    ekf.covariance = 70;
    uwfl2::LidarUpdateResult accepted;
    accepted.accepted = true;
    accepted.reason = uwfl2::LidarScanRejectionReason::kAccepted;
    transaction.finish(accepted);
    EXPECT_EQ(ekf.state, 30);
    EXPECT_EQ(ekf.covariance, 70);
}

TEST(ImuGapPolicy, DetectsOnlyConfiguredForwardDiscontinuities)
{
    EXPECT_FALSE(uwfl2::is_unobserved_imu_interval(10.0, 10.01, 0.1));
    EXPECT_TRUE(uwfl2::is_unobserved_imu_interval(10.0, 10.11, 0.1));
    EXPECT_FALSE(uwfl2::is_unobserved_imu_interval(10.0, 20.0, 0.0));
    EXPECT_FALSE(uwfl2::is_unobserved_imu_interval(20.0, 10.0, 0.1));
}

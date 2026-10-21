#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <vector>
#include <sensor_msgs/msg/imu.hpp>

#include "common_lib.h"
#include "sensor_parameter_utils.hpp"

namespace uwfl2
{

// The node consumes these queues while holding mtx_buffer. Prediction history
// has a separate lock; when both are needed, mtx_buffer is acquired first.
struct MappingInputBuffers
{
    using Clock = std::chrono::steady_clock;
    struct SonarWatermarkStatus
    {
        double safe_time = -1.0;
        double acquisition_age = 0.0;
        bool holding = false;
        bool missing = false;
        bool draining = false;
    };

    void ConfigureSonarStream(bool enabled, double frequency_hz);
    void NoteSonarReceipt(double timestamp,
                          Clock::time_point now = Clock::now());
    double PropagationWatermark(double latest_imu_time,
                                Clock::time_point now = Clock::now()) const;
    SonarWatermarkStatus SonarWatermark(
        double latest_imu_time,
        Clock::time_point now = Clock::now()) const;
    bool ImuTransportDraining(Clock::time_point now = Clock::now()) const;
    bool NavigationPacketPrecedesSonar(double packet_duration,
                                      double imu_period, double imu_timeout,
                                      double sonar_timeout) const;
    // Returns false when the acquisition timestamp is invalid, duplicated, or
    // older than the latest accepted sonar scan. Rejected scans never enter
    // the estimator queues.
    bool PushSonar(const PointCloudXYZI::Ptr &cloud, double timestamp);
    void PushImu(const sensor_msgs::msg::Imu::ConstSharedPtr &message);
    std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> PredictionSamples(
        double prediction_time, double correction_time);

    std::mutex mtx_buffer;
    std::mutex odometry_prediction_imu_mutex;
    std::deque<double> time_buffer;
    std::deque<PointCloudXYZI::Ptr> lidar_buffer;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> odometry_prediction_imu_buffer;
    double last_timestamp_lidar = 0.0;
    double last_timestamp_imu = -1.0;
    double first_timestamp_imu = -1.0;
    double last_processed_time = -1.0;
    double last_scan_end_time = -1.0;
    double lidar_mean_scantime = 0.0;
    int scan_num = 0;
    std::size_t sonar_received = 0;
    std::size_t sonar_late_dropped = 0;
    std::size_t sonar_nonmonotonic_dropped = 0;
    double max_sonar_acquisition_lag = 0.0;
    double max_sonar_late_age = 0.0;
    bool lidar_pushed = false;
    bool is_first_lidar = true;
    bool sonar_processing = false;
    bool sonar_stream_enabled = true;
    double sonar_missing_timeout = 0.5;
    std::deque<double> pending_sonar_timestamps;
    Clock::time_point last_sonar_receipt{};
    Clock::time_point last_imu_receipt{};
};

}  // namespace uwfl2

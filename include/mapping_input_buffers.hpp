#pragma once

#include <chrono>
#include <deque>
#include <mutex>
#include <vector>
#include <sensor_msgs/msg/imu.hpp>

#include "common_lib.h"

namespace uwfl2
{

// The node consumes these queues while holding mtx_buffer. Prediction history
// has a separate lock; when both are needed, mtx_buffer is acquired first.
struct MappingInputBuffers
{
    using Clock = std::chrono::steady_clock;
    void NoteSonarReceipt(Clock::time_point now = Clock::now());
    bool SonarReceptionTimedOut(double timeout_seconds,
                                Clock::duration callback_grace,
                                Clock::time_point now = Clock::now()) const;
    void PushSonar(const PointCloudXYZI::Ptr &cloud, double timestamp);
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
    double last_processed_time = -1.0;
    double last_scan_end_time = -1.0;
    double lidar_mean_scantime = 0.0;
    int scan_num = 0;
    bool lidar_pushed = false;
    bool is_first_lidar = true;
    bool sonar_processing = false;
    Clock::time_point last_sonar_receipt{};
};

}  // namespace uwfl2

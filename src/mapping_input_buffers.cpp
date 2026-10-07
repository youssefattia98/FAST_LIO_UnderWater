#include "mapping_input_buffers.hpp"

namespace uwfl2
{

std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> MappingInputBuffers::PredictionSamples(
    double prediction_time, double correction_time)
{
    std::lock_guard<std::mutex> lock(odometry_prediction_imu_mutex);
    if (odometry_prediction_imu_buffer.size() < 2U)
    {
        return {};
    }
    std::size_t begin = 0;
    while (begin + 1U < odometry_prediction_imu_buffer.size() &&
           get_time_sec(odometry_prediction_imu_buffer[begin + 1U]->header.stamp) <=
               prediction_time + 1e-9)
    {
        ++begin;
    }
    std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> samples(
        odometry_prediction_imu_buffer.begin() + begin,
        odometry_prediction_imu_buffer.end());
    // The main filter can still correct any epoch after its latest correction.
    // Pruning against prediction_time loses the history required for that reset.
    while (odometry_prediction_imu_buffer.size() > 2U &&
           get_time_sec(odometry_prediction_imu_buffer[1]->header.stamp) < correction_time)
    {
        odometry_prediction_imu_buffer.pop_front();
    }
    return samples;
}

void MappingInputBuffers::NoteSonarReceipt(Clock::time_point now)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    last_sonar_receipt = now;
    sonar_processing = true;
}

bool MappingInputBuffers::SonarReceptionTimedOut(
    double timeout_seconds, Clock::duration callback_grace, Clock::time_point now) const
{
    // Called under mtx_buffer: transport silence is not a sensor-time gap.
    return !sonar_processing &&
           now - last_sonar_receipt >=
               std::chrono::duration<double>(timeout_seconds) + callback_grace;
}

void MappingInputBuffers::PushSonar(const PointCloudXYZI::Ptr &cloud, double timestamp)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (!sonar_processing)
    {
        last_sonar_receipt = Clock::now();
    }
    sonar_processing = false;
    if (!is_first_lidar && timestamp < last_timestamp_lidar)
    {
        lidar_buffer.clear();
        time_buffer.clear();
        lidar_pushed = false;
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }
    lidar_buffer.push_back(cloud);
    time_buffer.push_back(timestamp);
    last_timestamp_lidar = timestamp;
}

void MappingInputBuffers::PushImu(const sensor_msgs::msg::Imu::ConstSharedPtr &message)
{
    const double timestamp = get_time_sec(message->header.stamp);
    mtx_buffer.lock();
    const bool timestamp_regressed = timestamp < last_timestamp_imu;
    if (timestamp_regressed)
    {
        imu_buffer.clear();
    }
    last_timestamp_imu = timestamp;
    imu_buffer.push_back(message);
    {
        std::lock_guard<std::mutex> prediction_lock(odometry_prediction_imu_mutex);
        if (timestamp_regressed)
        {
            odometry_prediction_imu_buffer.clear();
        }
        odometry_prediction_imu_buffer.push_back(message);
        while (odometry_prediction_imu_buffer.size() > 4000U)
        {
            odometry_prediction_imu_buffer.pop_front();
        }
    }
    mtx_buffer.unlock();
}

}  // namespace uwfl2

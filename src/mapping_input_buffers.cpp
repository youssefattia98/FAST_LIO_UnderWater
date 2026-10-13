#include "mapping_input_buffers.hpp"

namespace uwfl2
{

bool MappingInputBuffers::NavigationPacketPrecedesSonar(
    double packet_duration, double imu_period, double imu_timeout,
    double sonar_timeout) const
{
    if (lidar_pushed || lidar_buffer.empty() || time_buffer.empty() ||
        imu_buffer.empty() || last_processed_time <= 0.0)
        return false;
    if (last_scan_end_time > 0.0 &&
        time_buffer.front() - last_scan_end_time <= sonar_timeout)
        return false;
    const double first_imu = get_time_sec(imu_buffer.front()->header.stamp);
    const double begin = first_imu > last_processed_time + imu_timeout
        ? first_imu : last_processed_time;
    // Keep the entire acquisition interval with the scan, including the
    // nearest-sample rounding used by navigation packet selection.
    return begin + packet_duration + 0.5 * imu_period <
           time_buffer.front() - kAcquisitionTimeTolerance;
}

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

double MappingInputBuffers::PropagationWatermark(
    double latest_imu_time, Clock::time_point now) const
{
    // All modes share this transport budget, independent of sonar frequency.
    // The private predictor publishes fresh odometry while the main filter
    // retains acquisition history for slower sensor callbacks. Drain the tail
    // only after IMU transport stops, including after rosbag playback ends.
    constexpr double reorder_seconds = 0.02;
    constexpr auto drain_grace = std::chrono::milliseconds(250);
    return now - last_imu_receipt >= drain_grace
        ? latest_imu_time : latest_imu_time - reorder_seconds;
}

bool MappingInputBuffers::PushSonar(const PointCloudXYZI::Ptr &cloud, double timestamp)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (!sonar_processing)
    {
        last_sonar_receipt = Clock::now();
    }
    sonar_processing = false;
    ++sonar_received;
    if (!std::isfinite(timestamp) || timestamp < 0.0 ||
        (!is_first_lidar &&
         timestamp <= last_timestamp_lidar + kAcquisitionTimeTolerance))
    {
        ++sonar_nonmonotonic_dropped;
        return false;
    }
    max_sonar_acquisition_lag = std::max(
        max_sonar_acquisition_lag, last_timestamp_imu - timestamp);
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }
    lidar_buffer.push_back(cloud);
    time_buffer.push_back(timestamp);
    last_timestamp_lidar = timestamp;
    return true;
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
    last_imu_receipt = Clock::now();
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

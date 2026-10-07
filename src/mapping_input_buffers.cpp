#include "mapping_input_buffers.hpp"

namespace uwfl2
{

void MappingInputBuffers::PushSonar(const PointCloudXYZI::Ptr &cloud, double timestamp)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
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

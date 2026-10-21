#include "mapping_input_buffers.hpp"

namespace uwfl2
{

namespace
{
constexpr auto kTransportDrainGrace = std::chrono::milliseconds(250);
}

void MappingInputBuffers::ConfigureSonarStream(
    bool enabled, double frequency_hz)
{
    sonar_stream_enabled = enabled;
    const double valid_frequency =
        std::isfinite(frequency_hz) && frequency_hz > 0.0 ? frequency_hz : 4.0;
    sonar_missing_timeout =
        kSensorMissingAfterPeriods / valid_frequency;
}

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
    const double target = begin + packet_duration;
    const auto packet_tail = std::find_if(
        imu_buffer.begin(), imu_buffer.end(), [&](const auto &sample) {
            return get_time_sec(sample->header.stamp) >=
                   target - 0.5 * imu_period;
        });
    // Decide ownership using the actual queued sample, not the configured
    // nominal IMU period. Drivers commonly publish timestamped samples in
    // bursts; an estimated tail can appear to precede sonar while the next
    // real sample crosses its acquisition boundary, deadlocking both paths.
    return packet_tail != imu_buffer.end() &&
           get_time_sec((*packet_tail)->header.stamp) <
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

void MappingInputBuffers::NoteSonarReceipt(
    double timestamp, Clock::time_point now)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    last_sonar_receipt = now;
    if (std::isfinite(timestamp) && timestamp >= 0.0)
    {
        pending_sonar_timestamps.push_back(timestamp);
        sonar_processing = true;
    }
}

bool MappingInputBuffers::ImuTransportDraining(Clock::time_point now) const
{
    return last_imu_receipt != Clock::time_point{} &&
           now - last_imu_receipt >= kTransportDrainGrace;
}

MappingInputBuffers::SonarWatermarkStatus
MappingInputBuffers::SonarWatermark(
    double latest_imu_time, Clock::time_point now) const
{
    SonarWatermarkStatus result;
    result.safe_time = latest_imu_time;
    result.draining = ImuTransportDraining(now);
    if (!sonar_stream_enabled || result.draining || latest_imu_time < 0.0)
    {
        return result;
    }

    // A received scan can be in either of two stages: raw/preprocessing
    // (pending_sonar_timestamps) or converted/queued (time_buffer).  Both are
    // event barriers, but navigation may safely advance to the acquisition
    // start of the earliest one.  Ignoring time_buffer here leaves the
    // watermark at the previous completed scan and deadlocks the very
    // NavigationPacketPrecedesSonar path that is meant to bridge sparse scans.
    if (!pending_sonar_timestamps.empty() || !time_buffer.empty())
    {
        result.safe_time = latest_imu_time;
        if (!pending_sonar_timestamps.empty())
        {
            result.safe_time = std::min(
                result.safe_time,
                *std::min_element(pending_sonar_timestamps.begin(),
                                  pending_sonar_timestamps.end()));
        }
        if (!time_buffer.empty())
        {
            result.safe_time = std::min(result.safe_time,
                                        time_buffer.front());
        }
        result.holding = result.safe_time < latest_imu_time;
        return result;
    }

    if (last_timestamp_lidar > 0.0)
    {
        result.acquisition_age =
            std::max(0.0, latest_imu_time - last_timestamp_lidar);
        if (result.acquisition_age <= sonar_missing_timeout)
        {
            const double completed = last_scan_end_time > 0.0
                ? last_scan_end_time : last_timestamp_lidar;
            result.safe_time = std::min(latest_imu_time, completed);
            result.holding = result.safe_time < latest_imu_time;
        }
        else
        {
            result.missing = true;
        }
        return result;
    }

    if (first_timestamp_imu > 0.0 &&
        latest_imu_time - first_timestamp_imu <= sonar_missing_timeout)
    {
        result.safe_time = first_timestamp_imu;
        result.acquisition_age = latest_imu_time - first_timestamp_imu;
        result.holding = true;
    }
    else
    {
        result.missing = true;
    }
    return result;
}

double MappingInputBuffers::PropagationWatermark(
    double latest_imu_time, Clock::time_point now) const
{
    return SonarWatermark(latest_imu_time, now).safe_time;
}

bool MappingInputBuffers::PushSonar(const PointCloudXYZI::Ptr &cloud, double timestamp)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (pending_sonar_timestamps.empty())
    {
        last_sonar_receipt = Clock::now();
    }
    else
    {
        const auto match = std::find_if(
            pending_sonar_timestamps.begin(), pending_sonar_timestamps.end(),
            [timestamp](double pending) {
                return std::abs(pending - timestamp) <=
                       kAcquisitionTimeTolerance;
            });
        if (match != pending_sonar_timestamps.end())
        {
            pending_sonar_timestamps.erase(match);
        }
    }
    sonar_processing = !pending_sonar_timestamps.empty();
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
        first_timestamp_imu = timestamp;
    }
    if (first_timestamp_imu < 0.0)
    {
        first_timestamp_imu = timestamp;
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

#pragma once

#include <chrono>
#include <cmath>

namespace uwfl2
{

struct SensorTimestampDecision
{
    double timestamp = 0.0;
    bool used_arrival_time = false;
};

inline SensorTimestampDecision validate_sensor_timestamp(
    double sensor_timestamp, double arrival_timestamp,
    double maximum_future_skew_s)
{
    const bool valid_clock = std::isfinite(arrival_timestamp) &&
                             arrival_timestamp > 1.0;
    const bool impossible_future_stamp =
        maximum_future_skew_s > 0.0 && valid_clock &&
        std::isfinite(sensor_timestamp) &&
        sensor_timestamp - arrival_timestamp > maximum_future_skew_s;
    return impossible_future_stamp
               ? SensorTimestampDecision{arrival_timestamp, true}
               : SensorTimestampDecision{sensor_timestamp, false};
}

inline double system_arrival_timestamp()
{
    return std::chrono::duration<double>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

inline double usable_arrival_timestamp(double ros_timestamp)
{
    return std::isfinite(ros_timestamp) && ros_timestamp > 1.0
               ? ros_timestamp
               : system_arrival_timestamp();
}

}  // namespace uwfl2

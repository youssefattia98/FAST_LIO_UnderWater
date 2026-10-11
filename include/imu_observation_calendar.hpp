#pragma once

#include <deque>
#include <vector>
#include <sensor_msgs/msg/imu.hpp>
#include "common_lib.h"

namespace uwfl2
{
// Observation epochs follow acquisition time, never scan or callback boundaries.
class ImuObservationCalendar
{
public:
    using Sample = sensor_msgs::msg::Imu::ConstSharedPtr;
    struct Observation
    {
        double timestamp;
        std::deque<Sample> samples;
    };

    void Reset(double frontier)
    {
        samples_.clear();
        last_sample_ = frontier;
        next_epoch_ = frontier + kPeriod;
    }

    double NextEpoch() const { return next_epoch_; }

    std::vector<Observation> Take(const std::deque<Sample> &input, double end)
    {
        std::vector<Observation> result;
        for (const auto &sample : input)
        {
            if (!sample) continue;
            const double stamp = get_time_sec(sample->header.stamp);
            if (stamp > end + kAcquisitionTimeTolerance ||
                stamp <= last_sample_ + kAcquisitionTimeTolerance) continue;
            last_sample_ = stamp;
            samples_.push_back(sample);
            while (samples_.size() > 1 &&
                   get_time_sec(samples_.front()->header.stamp) < stamp - kPeriod - kAcquisitionTimeTolerance)
                samples_.pop_front();
            if (stamp + kAcquisitionTimeTolerance < next_epoch_) continue;
            result.push_back({stamp, samples_});
            // Missed acquisition epochs are not fabricated or fused repeatedly.
            do { next_epoch_ += kPeriod; } while (next_epoch_ <= stamp + kAcquisitionTimeTolerance);
        }
        return result;
    }

private:
    static constexpr double kPeriod = 0.2;
    double last_sample_ = -1.0;
    double next_epoch_ = 0.0;
    std::deque<Sample> samples_;
};
}  // namespace uwfl2

#pragma once

#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "auxiliary_sensor_fusion.hpp"

namespace uwfl2
{
// Build-only characterization: no ROS output or estimator feedback.
class ReplayTrace
{
public:
    ReplayTrace()
    {
        if (const char *path = std::getenv("UWFL2_REPLAY_TRACE_PATH"))
        {
            output_.open(path);
            if (!output_) throw std::runtime_error("Cannot open replay trace");
            sonar_input_.open(std::string(path) + ".sonar.csv");
            if (!sonar_input_) throw std::runtime_error("Cannot open sonar receipt trace");
            sonar_input_ << "stamp_ns,point_count\n";
            output_ << std::setprecision(17);
            output_ << "stage,time,epoch,count,key,px,py,pz";
            for (int i = 0; i < 9; ++i) output_ << ",r" << i;
            for (const char *name : {"vel", "bg", "ba", "grav", "bdvl"})
                for (int i = 0; i < 3; ++i) output_ << ',' << name << i;
            output_ << ",bp,cov_hash";
            for (int i = 0; i < state_ikfom::DOF; ++i) output_ << ",cov" << i;
            output_ << ",attitude_ba_cross,attitude_grav_cross,ba_grav_cross\n";
        }
    }

    void SonarReceived(const sensor_msgs::msg::PointCloud2 &message)
    {
        // Only the mutually exclusive sonar callback writes this separate file.
        if (sonar_input_)
            sonar_input_ << static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
                message.header.stamp.nanosec << ',' <<
                static_cast<std::size_t>(message.width) * message.height << '\n';
    }

    static std::uint64_t Fingerprint(const sensor_msgs::msg::Imu &message)
    {
        const double values[]{message.angular_velocity.x, message.angular_velocity.y,
            message.angular_velocity.z, message.linear_acceleration.x,
            message.linear_acceleration.y, message.linear_acceleration.z,
            message.orientation.x, message.orientation.y, message.orientation.z,
            message.orientation.w};
        return HeaderFingerprint(message.header) ^
            Hash(reinterpret_cast<const unsigned char *>(values), sizeof(values)) ^
            ArrayFingerprint(message.angular_velocity_covariance) ^
            ArrayFingerprint(message.linear_acceleration_covariance) ^
            ArrayFingerprint(message.orientation_covariance);
    }

    static std::uint64_t Fingerprint(const AuxiliarySensorFusion::DvlMsg &message)
    {
        const auto &v = message.twist.twist;
        const double values[]{v.linear.x, v.linear.y, v.linear.z,
                              v.angular.x, v.angular.y, v.angular.z};
        return HeaderFingerprint(message.header) ^
            Hash(reinterpret_cast<const unsigned char *>(values), sizeof(values)) ^
            ArrayFingerprint(message.twist.covariance);
    }

    static std::uint64_t Fingerprint(const AuxiliarySensorFusion::PressureMsg &message)
    {
        const double values[]{message.fluid_pressure, message.variance};
        return HeaderFingerprint(message.header) ^
            Hash(reinterpret_cast<const unsigned char *>(values), sizeof(values));
    }

    static std::uint64_t Fingerprint(const AuxiliarySensorFusion::MagMsg &message)
    {
        const double values[]{message.magnetic_field.x, message.magnetic_field.y,
                              message.magnetic_field.z};
        return HeaderFingerprint(message.header) ^
            Hash(reinterpret_cast<const unsigned char *>(values), sizeof(values)) ^
            ArrayFingerprint(message.magnetic_field_covariance);
    }

    static std::uint64_t ImuFingerprint(
        const std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> &messages)
    {
        std::uint64_t result = 0;
        for (const auto &message : messages)
            result = result * 1099511628211ULL ^ Fingerprint(*message);
        return result;
    }

    static std::uint64_t MeasurementFingerprint(
        const AuxiliarySensorFusion::TimedMeasurement &measurement)
    {
        if (measurement.dvl) return Fingerprint(*measurement.dvl);
        if (measurement.pressure) return Fingerprint(*measurement.pressure);
        if (measurement.magnetometer) return Fingerprint(*measurement.magnetometer);
        return 0;
    }

    template<class PointRange>
    static std::uint64_t PointFingerprint(const PointRange &points)
    {
        std::uint64_t result = 0;
        for (const auto &point : points)
        {
            const float coordinates[]{point.x, point.y, point.z};
            result = result * 1099511628211ULL ^
                Hash(reinterpret_cast<const unsigned char *>(coordinates), sizeof(coordinates));
        }
        return result;
    }

    template<class NeighborRange>
    static std::uint64_t NeighborFingerprint(const NeighborRange &neighbors)
    {
        std::uint64_t result = 0;
        for (const auto &points : neighbors)
            result = result * 1099511628211ULL ^ PointFingerprint(points);
        return result;
    }

    template<class Filter>
    void Record(const char *stage, double timestamp, double epoch,
                std::size_t count, std::uint64_t key, const Filter &filter)
    {
        if (!output_) return;
        const auto state = filter.get_x();
        const auto covariance = filter.get_P();
        output_ << stage << ',' << timestamp << ',' << epoch << ',' << count << ',' << key;
        for (int i = 0; i < 3; ++i) output_ << ',' << state.pos[i];
        const auto rotation = state.rot.toRotationMatrix();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) output_ << ',' << rotation(i, j);
        for (const auto &vector : {V3D(state.vel), V3D(state.bg), V3D(state.ba),
                                  V3D(state.grav[0], state.grav[1], state.grav[2]),
                                  V3D(state.b_dvl)})
            for (int i = 0; i < 3; ++i) output_ << ',' << vector[i];
        output_ << ',' << state.b_pressure << ','
                << Hash(reinterpret_cast<const unsigned char *>(covariance.data()),
                        sizeof(double) * covariance.size());
        for (int i = 0; i < state_ikfom::DOF; ++i) output_ << ',' << covariance(i, i);
        output_ << ',' << covariance.template block<3, 3>(3, 18).norm()
                << ',' << covariance.template block<3, 2>(3, 21).norm()
                << ',' << covariance.template block<3, 2>(18, 21).norm() << '\n';
    }

private:
    template<class Array>
    static std::uint64_t ArrayFingerprint(const Array &values)
    {
        return Hash(reinterpret_cast<const unsigned char *>(values.data()),
                    sizeof(values[0]) * values.size());
    }

    static std::uint64_t HeaderFingerprint(const std_msgs::msg::Header &header)
    {
        const std::int64_t stamp =
            static_cast<std::int64_t>(header.stamp.sec) * 1000000000LL + header.stamp.nanosec;
        return Hash(reinterpret_cast<const unsigned char *>(&stamp), sizeof(stamp)) ^
            Hash(reinterpret_cast<const unsigned char *>(header.frame_id.data()),
                 header.frame_id.size());
    }

    static std::uint64_t Hash(const unsigned char *data, std::size_t size)
    {
        std::uint64_t hash = 14695981039346656037ULL;
        for (std::size_t i = 0; i < size; ++i)
            hash = (hash ^ data[i]) * 1099511628211ULL;
        return hash;
    }
    std::ofstream output_;
    std::ofstream sonar_input_;
};
}  // namespace uwfl2

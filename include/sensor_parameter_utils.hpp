#pragma once

#include <cmath>
#include <Eigen/Geometry>
#include <stdexcept>
#include <vector>

namespace uwfl2
{

// Sensor-to-body rotation: Rz(yaw) * Ry(pitch) * Rx(roll), FLU axes.
inline Eigen::Matrix3d rotation_from_rpy_degrees(const std::vector<double> &rpy)
{
    if (rpy.size() != 3 || !std::isfinite(rpy[0]) ||
        !std::isfinite(rpy[1]) || !std::isfinite(rpy[2]))
        throw std::invalid_argument("rotation must contain three finite RPY angles in degrees");
    constexpr double radians_per_degree = 0.017453292519943295;
    return (Eigen::AngleAxisd(rpy[2] * radians_per_degree, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy[1] * radians_per_degree, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy[0] * radians_per_degree, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

inline double timeout_from_frequency(double frequency_hz,
                                     double fallback_frequency_hz)
{
    const double valid_fallback =
        std::isfinite(fallback_frequency_hz) && fallback_frequency_hz > 0.0
            ? fallback_frequency_hz
            : 1.0;
    const double frequency =
        std::isfinite(frequency_hz) && frequency_hz > 0.0
            ? frequency_hz
            : valid_fallback;
    return 1.0 / frequency;
}

}  // namespace uwfl2

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>
#include <Eigen/Eigenvalues>

namespace uwfl2
{
struct SonarInputMetrics
{
    std::size_t finite_points = 0;
    double azimuth_span = 0.0, elevation_span = 0.0, median_range = 0.0;
};

inline double SonarQuantile(std::vector<double> values, double fraction)
{
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double index = fraction * (values.size() - 1);
    const auto lower = static_cast<std::size_t>(index);
    const auto upper = std::min(lower + 1, values.size() - 1);
    return values[lower] + (index - lower) * (values[upper] - values[lower]);
}

// Sensor-frame input, before deskew. Bound angular-statistics storage while
// counting every finite nonzero return; circular centering avoids +/-pi.
template<class Points>
SonarInputMetrics MeasureSonarInput(const Points &points)
{
    SonarInputMetrics metrics;
    std::vector<double> azimuth, elevation, ranges;
    constexpr std::size_t sample_limit = 2048;
    const std::size_t stride = std::max<std::size_t>(1, (points.size() + sample_limit - 1) / sample_limit);
    double sin_sum = 0.0, cos_sum = 0.0;
    std::size_t index = 0;
    for (const auto &point : points)
    {
        const double x = point.x, y = point.y, z = point.z;
        const double range = std::hypot(std::hypot(x, y), z);
        if (!std::isfinite(range) || range <= 0.0) { ++index; continue; }
        ++metrics.finite_points;
        if (index++ % stride != 0) continue;
        const double angle = std::atan2(y, x);
        azimuth.push_back(angle);
        elevation.push_back(std::atan2(z, std::hypot(x, y)));
        ranges.push_back(range);
        sin_sum += std::sin(angle); cos_sum += std::cos(angle);
    }
    const double center = std::atan2(sin_sum, cos_sum);
    for (double &angle : azimuth)
        angle = std::atan2(std::sin(angle - center), std::cos(angle - center));
    metrics.azimuth_span = SonarQuantile(azimuth, 0.95) - SonarQuantile(azimuth, 0.05);
    metrics.elevation_span = SonarQuantile(elevation, 0.95) - SonarQuantile(elevation, 0.05);
    metrics.median_range = SonarQuantile(ranges, 0.5);
    return metrics;
}

enum class SonarInputCondition { Calibrating, Healthy, Empty, Sparse, Cropped, Missing };
inline const char *SonarConditionName(SonarInputCondition condition)
{
    switch (condition)
    {
        case SonarInputCondition::Calibrating: return "calibrating";
        case SonarInputCondition::Healthy: return "healthy_input";
        case SonarInputCondition::Empty: return "empty";
        case SonarInputCondition::Sparse: return "sparse";
        case SonarInputCondition::Cropped: return "reduced_coverage";
        case SonarInputCondition::Missing: return "missing";
    }
    return "calibrating";
}
struct SonarDetection
{
    SonarInputCondition condition = SonarInputCondition::Calibrating;
    double timestamp = -1.0, count_ratio = 1.0, azimuth_ratio = 1.0, elevation_ratio = 1.0;
    double acquisition_age = 0.0;
};

class SonarAvailability
{
public:
    explicit SonarAvailability(double frequency = 6.0) { SetFrequency(frequency); }
    void SetFrequency(double frequency)
    {
        if (!std::isfinite(frequency) || frequency <= 0.0)
            throw std::invalid_argument("Sonar frequency must be positive and finite");
        missing_deadline_ = 2.0 / frequency + reorder_allowance;
    }
    SonarDetection Observe(double timestamp, const SonarInputMetrics &metrics)
    {
        if (!std::isfinite(timestamp) || timestamp < 0.0 || timestamp < last_timestamp_)
            throw std::invalid_argument("Non-monotonic sonar acquisition timestamp");
        last_timestamp_ = timestamp;
        last_ = SonarDetection{}; last_.timestamp = timestamp;
        if (metrics.finite_points == 0)
        {
            last_.condition = SonarInputCondition::Empty;
            return last_;
        }
        if (reference_.size() < reference_samples)
        {
            reference_.push_back(metrics);
            if (reference_.size() == reference_samples)
            {
                std::vector<double> points, azimuth, elevation;
                for (const auto &sample : reference_)
                {
                    points.push_back(sample.finite_points);
                    azimuth.push_back(sample.azimuth_span); elevation.push_back(sample.elevation_span);
                }
                reference_points_ = SonarQuantile(points, 0.5);
                reference_azimuth_ = SonarQuantile(azimuth, 0.5);
                reference_elevation_ = SonarQuantile(elevation, 0.5);
            }
            return last_;
        }
        last_.count_ratio = metrics.finite_points / reference_points_;
        last_.azimuth_ratio = reference_azimuth_ > 1e-6 ? metrics.azimuth_span / reference_azimuth_ : 1.0;
        last_.elevation_ratio = reference_elevation_ > 1e-6 ? metrics.elevation_span / reference_elevation_ : 1.0;
        // Fixed reporting ratios, not scan-rejection thresholds. No GT or
        // injected-fault window is available to this class.
        if (std::min(last_.azimuth_ratio, last_.elevation_ratio) < 0.5)
            last_.condition = SonarInputCondition::Cropped;
        else if (last_.count_ratio < 0.25) last_.condition = SonarInputCondition::Sparse;
        else last_.condition = SonarInputCondition::Healthy;
        return last_;
    }
    SonarDetection At(double timestamp) const
    {
        auto detection = last_; detection.timestamp = timestamp;
        detection.acquisition_age = last_timestamp_ >= 0.0
            ? std::max(0.0, timestamp - last_timestamp_) : std::numeric_limits<double>::infinity();
        if (last_timestamp_ < 0.0 || detection.acquisition_age > missing_deadline_)
            detection.condition = SonarInputCondition::Missing;
        return detection;
    }
    double MissingDeadline() const { return missing_deadline_; }
    static constexpr double reorder_allowance = 0.02;
    static constexpr std::size_t reference_samples = 20;
private:
    std::vector<SonarInputMetrics> reference_;
    double reference_points_ = 1.0, reference_azimuth_ = 0.0, reference_elevation_ = 0.0;
    double last_timestamp_ = -1.0, missing_deadline_ = 0.0;
    SonarDetection last_;
};

// Scale attitude columns by range for comparable units; report rank only.
// This must not project the gain or demand complete six-direction information.
inline int SonarGeometryRank(const Eigen::MatrixXd &jacobian, double typical_range)
{
    if (jacobian.rows() == 0 || jacobian.cols() < 6 || !jacobian.allFinite()) return 0;
    Eigen::MatrixXd scaled = jacobian.leftCols(6);
    scaled.rightCols(3) /= std::max(typical_range, 1.0);
    const Eigen::Matrix<double, 6, 6> information = scaled.transpose() * scaled;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(information);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues().maxCoeff() <= 0.0) return 0;
    return (eigen.eigenvalues().array() > eigen.eigenvalues().maxCoeff() * 1e-6).count();
}
}  // namespace uwfl2

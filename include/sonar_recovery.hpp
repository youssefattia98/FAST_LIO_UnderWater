#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <Eigen/Cholesky>

namespace uwfl2
{
// Engineering recovery policy, not a GenZ-LIO measurement model. All pose
// coordinates and their covariance are expressed in the same local SE(3) tangent.
class SonarRecovery
{
public:
    using PoseDelta = Eigen::Matrix<double, 6, 1>;
    using PoseCovariance = Eigen::Matrix<double, 6, 6>;
    enum class Decision { Normal, Validating, Inconsistent, Resumed };

    void RequireValidation()
    {
        recovering_ = true;
        consecutive_ = 0;
    }

    Decision Candidate(std::uint64_t generation, const PoseDelta &delta,
                       const PoseCovariance &covariance, double overlap,
                       double mean_squared_whitened_residual)
    {
        if (!recovering_) return Decision::Normal;
        if (generation != generation_)
        {
            consecutive_ = 0;
            generation_ = generation;
        }
        // Freeze these settings before replay: >=10% supported returns, mean
        // residual within three measurement sigmas, and a six-coordinate
        // prediction-consistency bound. No full-rank sonar geometry is required.
        const bool valid = std::isfinite(overlap) && overlap >= 0.1 && overlap <= 1.0 &&
            std::isfinite(mean_squared_whitened_residual) &&
            mean_squared_whitened_residual >= 0.0 && mean_squared_whitened_residual <= 9.0 &&
            Consistent(delta, covariance) &&
            (consecutive_ == 0 || Consistent(delta - previous_delta_, covariance + previous_covariance_));
        if (!valid)
        {
            consecutive_ = 0;
            return Decision::Inconsistent;
        }
        previous_delta_ = delta;
        previous_covariance_ = covariance;
        if (++consecutive_ < 3) return Decision::Validating;
        recovering_ = false;
        return Decision::Resumed;
    }

    bool Recovering() const { return recovering_; }
    std::size_t Consecutive() const { return consecutive_; }

private:
    static bool Consistent(const PoseDelta &delta, const PoseCovariance &covariance)
    {
        if (!delta.allFinite() || !covariance.allFinite() ||
            (covariance - covariance.transpose()).norm() > 1e-8) return false;
        const Eigen::LDLT<PoseCovariance> factor(covariance);
        if (factor.info() != Eigen::Success || factor.vectorD().minCoeff() <= 0.0) return false;
        const double squared = delta.dot(factor.solve(delta));
        return std::isfinite(squared) && squared >= 0.0 && squared <= 6.0 * 9.0;
    }

    bool recovering_ = false;
    std::size_t consecutive_ = 0;
    std::uint64_t generation_ = 0;
    PoseDelta previous_delta_ = PoseDelta::Zero();
    PoseCovariance previous_covariance_ = PoseCovariance::Zero();
};
}  // namespace uwfl2

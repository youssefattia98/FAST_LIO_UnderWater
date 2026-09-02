#pragma once

#include <algorithm>
#include <cstddef>
#include <type_traits>
#include <utility>

namespace uwfl2
{

enum class LidarScanRejectionReason
{
    kAccepted,
    kSparseInput,
    kInsufficientCorrespondences,
    kUpdateNotApplied,
};

inline const char *lidar_scan_status(LidarScanRejectionReason reason)
{
    switch (reason)
    {
        case LidarScanRejectionReason::kAccepted:
            return "lidar_update";
        case LidarScanRejectionReason::kSparseInput:
            return "sparse_input";
        case LidarScanRejectionReason::kInsufficientCorrespondences:
            return "insufficient_effective_features";
        case LidarScanRejectionReason::kUpdateNotApplied:
            return "lidar_update_not_applied";
    }
    return "unknown_lidar_update";
}

struct LidarUpdateResult
{
    bool accepted = false;
    LidarScanRejectionReason reason =
        LidarScanRejectionReason::kUpdateNotApplied;
    std::size_t input_points = 0;
    std::size_t effective_features = 0;
    std::size_t map_points_inserted = 0;
};

class LidarScanQualityPolicy
{
public:
    explicit LidarScanQualityPolicy(std::size_t minimum_scan_points = 5,
                                    std::size_t minimum_effective_features = 1)
        : minimum_scan_points_(minimum_scan_points),
          minimum_effective_features_(minimum_effective_features)
    {
    }

    bool input_is_sufficient(std::size_t points) const
    {
        return points >= minimum_scan_points_;
    }

    bool map_initialization_is_sufficient(std::size_t points) const
    {
        // FAST-LIO2 historically required more than five points to seed a map.
        return points >= std::max<std::size_t>(minimum_scan_points_, 6);
    }

    bool features_are_sufficient(std::size_t features) const
    {
        return features >= minimum_effective_features_;
    }

    LidarUpdateResult reject_sparse_input(std::size_t points) const
    {
        return {false, LidarScanRejectionReason::kSparseInput, points, 0, 0};
    }

    LidarUpdateResult accept_map_initialization(std::size_t points) const
    {
        return {true, LidarScanRejectionReason::kAccepted, points, 0, points};
    }

    LidarUpdateResult evaluate_update(std::size_t points,
                                      std::size_t effective_features,
                                      bool update_applied) const
    {
        if (!input_is_sufficient(points))
        {
            return reject_sparse_input(points);
        }
        if (!features_are_sufficient(effective_features))
        {
            return {false,
                    LidarScanRejectionReason::kInsufficientCorrespondences,
                    points, effective_features, 0};
        }
        if (!update_applied)
        {
            return {false, LidarScanRejectionReason::kUpdateNotApplied,
                    points, effective_features, 0};
        }
        return {true, LidarScanRejectionReason::kAccepted, points,
                effective_features, 0};
    }

private:
    std::size_t minimum_scan_points_;
    std::size_t minimum_effective_features_;
};

inline bool should_insert_lidar_scan(const LidarUpdateResult &result)
{
    return result.accepted;
}

template <typename Ekf>
class LidarUpdateTransaction
{
public:
    using State = std::decay_t<decltype(std::declval<Ekf &>().get_x())>;
    using Covariance = std::decay_t<decltype(std::declval<Ekf &>().get_P())>;

    explicit LidarUpdateTransaction(Ekf &ekf)
        : ekf_(ekf), state_before_(ekf.get_x()), covariance_before_(ekf.get_P())
    {
    }

    void finish(const LidarUpdateResult &result)
    {
        if (result.accepted)
        {
            return;
        }
        ekf_.change_x(state_before_);
        ekf_.change_P(covariance_before_);
    }

private:
    Ekf &ekf_;
    State state_before_;
    Covariance covariance_before_;
};

}  // namespace uwfl2

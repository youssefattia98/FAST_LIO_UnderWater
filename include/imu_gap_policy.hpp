#pragma once

#include <cmath>

namespace uwfl2
{

inline bool is_unobserved_imu_interval(double begin, double end,
                                       double maximum_gap_s)
{
    return maximum_gap_s > 0.0 && std::isfinite(begin) &&
           std::isfinite(end) && end - begin > maximum_gap_s;
}

}  // namespace uwfl2

#pragma once

#include <cmath>

namespace uwfl2
{

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

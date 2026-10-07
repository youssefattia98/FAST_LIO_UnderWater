#include <gtest/gtest.h>

#include <limits>

#include "sensor_parameter_utils.hpp"

TEST(SensorParameterUtils, ConvertsFrequencyToOnePeriodTimeout)
{
    EXPECT_DOUBLE_EQ(uwfl2::timeout_from_frequency(4.0, 1.0), 0.25);
    EXPECT_DOUBLE_EQ(uwfl2::timeout_from_frequency(15.0, 1.0), 1.0 / 15.0);
}

TEST(SensorParameterUtils, UsesFallbackForInvalidFrequency)
{
    EXPECT_DOUBLE_EQ(uwfl2::timeout_from_frequency(0.0, 5.0), 0.2);
    EXPECT_DOUBLE_EQ(
        uwfl2::timeout_from_frequency(
            std::numeric_limits<double>::quiet_NaN(), 10.0),
        0.1);
}

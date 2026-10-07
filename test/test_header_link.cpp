#include "common_lib.h"
#include "common_lib.h"
#include "IMU_Processing.hpp"
#include "IMU_Processing.hpp"
#include "auxiliary_sensor_fusion.hpp"
#include "header_link_fixture.hpp"

#include <gtest/gtest.h>

TEST(SharedHeaders, NumericalValuesAgreeAcrossTranslationUnits)
{
    const auto expected = std::array<double, 8>{1, 0.2, 1, -1, 0.0001, 13, 12.25, 0};
    EXPECT_EQ(SharedHeaderValues(), expected);
    EXPECT_EQ(SharedHeaderValuesFromOtherTranslationUnit(), expected);
    EXPECT_EQ(state_ikfom::DOF, 27);
    EXPECT_EQ(process_noise_ikfom::DOF, 12);
}

TEST(SharedHeaders, InlineGlobalsHaveOneAddressAcrossTranslationUnits)
{
    EXPECT_EQ(&Eye3d, SharedIdentityFromOtherTranslationUnit());
    EXPECT_TRUE(Eye3d.isIdentity());
    EXPECT_TRUE(Eye3f.isIdentity());
    EXPECT_TRUE(Zero3d.isZero());
    EXPECT_TRUE(Zero3f.isZero());
}

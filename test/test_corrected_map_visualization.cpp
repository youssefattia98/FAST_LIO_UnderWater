#include <gtest/gtest.h>

#include <limits>

#include "corrected_map_visualization.hpp"

TEST(CorrectedMapDisplayVoxels, SuppressesRepeatedPointsInOneCell)
{
    uwfl2::CorrectedMapDisplayVoxels voxels(0.03);
    EXPECT_TRUE(voxels.insert(1.000, 2.000, 3.000));
    EXPECT_FALSE(voxels.insert(1.005, 2.005, 3.005));
    EXPECT_EQ(voxels.size(), 1U);
}

TEST(CorrectedMapDisplayVoxels, RetainsNewAndNegativeCells)
{
    uwfl2::CorrectedMapDisplayVoxels voxels(0.03);
    EXPECT_TRUE(voxels.insert(0.029, 0.0, 0.0));
    EXPECT_TRUE(voxels.insert(0.031, 0.0, 0.0));
    EXPECT_TRUE(voxels.insert(-0.001, 0.0, 0.0));
    EXPECT_EQ(voxels.size(), 3U);
}

TEST(CorrectedMapDisplayVoxels, ClearSupportsCorrectedMapRebuild)
{
    uwfl2::CorrectedMapDisplayVoxels voxels;
    EXPECT_TRUE(voxels.insert(1.0, 2.0, 3.0));
    voxels.clear();
    EXPECT_EQ(voxels.size(), 0U);
    EXPECT_TRUE(voxels.insert(1.0, 2.0, 3.0));
}

TEST(CorrectedMapDisplayVoxels, RejectsNonFinitePoints)
{
    uwfl2::CorrectedMapDisplayVoxels voxels;
    EXPECT_FALSE(voxels.insert(
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
    EXPECT_FALSE(voxels.insert(
        0.0, std::numeric_limits<double>::infinity(), 0.0));
    EXPECT_EQ(voxels.size(), 0U);
}

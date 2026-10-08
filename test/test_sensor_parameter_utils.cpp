#include <gtest/gtest.h>

#include <limits>
#include <algorithm>
#include <fstream>
#include <future>

#include "sensor_parameter_utils.hpp"
#include "map_file_utils.hpp"

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

TEST(SensorParameterUtils, RotationUsesRollPitchYawDegreesAndSensorToBodyDirection)
{
    EXPECT_TRUE(uwfl2::rotation_from_rpy_degrees({0.0, 0.0, 0.0})
                    .isApprox(Eigen::Matrix3d::Identity()));
    const auto rotation = uwfl2::rotation_from_rpy_degrees({20.0, -35.0, 70.0});
    constexpr double radians = 0.017453292519943295;
    const Eigen::Matrix3d expected =
        (Eigen::AngleAxisd(70.0 * radians, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(-35.0 * radians, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(20.0 * radians, Eigen::Vector3d::UnitX())).toRotationMatrix();
    EXPECT_TRUE(rotation.isApprox(expected, 1e-14));
    EXPECT_NEAR(rotation.determinant(), 1.0, 1e-14);
    EXPECT_TRUE((uwfl2::rotation_from_rpy_degrees({0.0, 0.0, 90.0}) *
                 Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitY(), 1e-14));
    EXPECT_THROW(uwfl2::rotation_from_rpy_degrees({1.0, 2.0}), std::invalid_argument);
    EXPECT_THROW(uwfl2::rotation_from_rpy_degrees(
        {0.0, std::numeric_limits<double>::quiet_NaN(), 0.0}), std::invalid_argument);
}

TEST(MapFiles, ReservesUniqueNamesWithoutOverwritingAndCreatesDirectories)
{
    const auto root = std::filesystem::temp_directory_path() /
        ("uwfl2_map_save_" + std::to_string(::getpid()));
    std::filesystem::remove_all(root);
    const auto requested = root / "nested" / "test.pcd";
    EXPECT_EQ(uwfl2::reserve_map_path(requested), requested);
    std::ofstream(requested) << "previous map";
    EXPECT_EQ(uwfl2::reserve_map_path(requested), requested.parent_path() / "test(1).pcd");
    EXPECT_EQ(uwfl2::reserve_map_path(requested), requested.parent_path() / "test(2).pcd");
    std::string preserved;
    std::ifstream input(requested);
    std::getline(input, preserved);
    EXPECT_EQ(preserved, "previous map");
    std::vector<std::future<std::filesystem::path>> writers;
    for (int i = 0; i < 4; ++i)
        writers.push_back(std::async(std::launch::async, [requested] {
            return uwfl2::reserve_map_path(requested);
        }));
    std::vector<std::filesystem::path> paths;
    for (auto &writer : writers) paths.push_back(writer.get());
    std::sort(paths.begin(), paths.end());
    EXPECT_EQ(std::unique(paths.begin(), paths.end()), paths.end());
    EXPECT_THROW(uwfl2::reserve_map_path(requested / "invalid.pcd"), std::filesystem::filesystem_error);
    std::filesystem::remove_all(root);
}

TEST(SensorParameterUtils, WorldFrameConversionPreservesExistingNormalizedTransform)
{
    Eigen::Matrix3d legacy;
    legacy << 0.7364, -0.6765, -0.0117,
              0.6763, 0.7365, -0.0142,
              0.0182, 0.0025, 0.9998;
    const auto expected = Eigen::Quaterniond(legacy).normalized().toRotationMatrix();
    const auto migrated = uwfl2::rotation_from_rpy_degrees(
        {0.144763019760591, -1.04299280234362, 42.5657626672732});
    EXPECT_TRUE(migrated.isApprox(expected, 1e-13));
}

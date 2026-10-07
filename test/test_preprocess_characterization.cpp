#include <gtest/gtest.h>
#include <cstring>
#include <limits>
#include "preprocess.h"

TEST(PreprocessCharacterization, PaddedRowsAndRangeFilterPreserveIntensity)
{
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->width = 2;
    cloud->height = 2;
    cloud->point_step = 16;
    cloud->row_step = 40;
    cloud->data.resize(80, 0);
    for (int i = 0; i < 4; ++i)
    {
        sensor_msgs::msg::PointField field;
        field.name = std::vector<std::string>{"x", "y", "z", "intensity"}[i];
        field.offset = i * 4;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        cloud->fields.push_back(field);
    }
    for (int row = 0; row < 2; ++row)
        for (int column = 0; column < 2; ++column)
        {
            const float point[4] = {static_cast<float>(row * 2 + column), 0, 0, 17};
            std::memcpy(cloud->data.data() + row * cloud->row_step + column * cloud->point_step,
                        point, sizeof(point));
        }
    Preprocess preprocess;
    preprocess.blind = 0.2;
    PointCloudXYZI::Ptr output(new PointCloudXYZI());
    preprocess.process(cloud, output);
    ASSERT_EQ(output->size(), 3u);
    for (std::size_t i = 0; i < output->size(); ++i)
    {
        EXPECT_EQ((*output)[i].x, i + 1);
        EXPECT_EQ((*output)[i].intensity, 17);
        EXPECT_EQ((*output)[i].curvature, 0);
    }
}

namespace
{
template<typename T>
void CheckDatatype(uint8_t datatype)
{
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = "fixture_sonar";
    cloud->header.stamp.sec = 17;
    cloud->width = 1;
    cloud->height = 1;
    cloud->point_step = 4 * sizeof(T);
    cloud->row_step = cloud->point_step;
    cloud->data.resize(cloud->row_step);
    const T values[] = {T(7), T(3), T(2), T(1)};
    std::memcpy(cloud->data.data(), values, sizeof(values));
    const std::vector<std::string> names = {"intensity", "z", "y", "x"};
    for (std::size_t index = 0; index < names.size(); ++index)
    {
        sensor_msgs::msg::PointField field;
        field.name = names[index];
        field.offset = index * sizeof(T);
        field.datatype = datatype;
        field.count = 1;
        cloud->fields.push_back(field);
    }
    Preprocess preprocess;
    PointCloudXYZI::Ptr output(new PointCloudXYZI());
    preprocess.process(cloud, output);
    ASSERT_EQ(output->size(), 1u);
    EXPECT_EQ(output->front().x, 1);
    EXPECT_EQ(output->front().y, 2);
    EXPECT_EQ(output->front().z, 3);
    EXPECT_EQ(output->front().intensity, 7);
    EXPECT_EQ(output->front().normal_x, 0);
    EXPECT_EQ(output->front().normal_y, 0);
    EXPECT_EQ(output->front().normal_z, 0);
    EXPECT_EQ(output->front().curvature, 0);
    EXPECT_EQ(output->width, 1u);
    EXPECT_EQ(output->height, 1u);
    EXPECT_TRUE(output->is_dense);
    // The inherited decoder does not transfer the ROS header to the PCL output.
    EXPECT_TRUE(output->header.frame_id.empty());
    EXPECT_EQ(output->header.stamp, 0u);
    cloud->fields.erase(cloud->fields.begin());
    preprocess.process(cloud, output);
    ASSERT_EQ(output->size(), 1u);
    EXPECT_EQ(output->front().intensity, std::numeric_limits<float>::max());
    cloud->width = 0;
    preprocess.process(cloud, output);
    EXPECT_TRUE(output->empty());
    EXPECT_EQ(output->width, 0u);
    EXPECT_EQ(output->height, 0u);
}
}

TEST(PreprocessCharacterization, AllSupportedDatatypesAndMetadata)
{
    using Field = sensor_msgs::msg::PointField;
    CheckDatatype<int8_t>(Field::INT8);
    CheckDatatype<uint8_t>(Field::UINT8);
    CheckDatatype<int16_t>(Field::INT16);
    CheckDatatype<uint16_t>(Field::UINT16);
    CheckDatatype<int32_t>(Field::INT32);
    CheckDatatype<uint32_t>(Field::UINT32);
    CheckDatatype<float>(Field::FLOAT32);
    CheckDatatype<double>(Field::FLOAT64);
}

TEST(PreprocessCharacterization, MissingCoordinatesProduceEmptyCloud)
{
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    Preprocess preprocess;
    PointCloudXYZI::Ptr output(new PointCloudXYZI());
    preprocess.process(cloud, output);
    EXPECT_TRUE(output->empty());
}

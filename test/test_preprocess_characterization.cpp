#include <gtest/gtest.h>
#include <cstring>
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
    preprocess.time_unit = US;
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

TEST(PreprocessCharacterization, MissingCoordinatesProduceEmptyCloud)
{
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    Preprocess preprocess;
    preprocess.time_unit = US;
    PointCloudXYZI::Ptr output(new PointCloudXYZI());
    preprocess.process(cloud, output);
    EXPECT_TRUE(output->empty());
}

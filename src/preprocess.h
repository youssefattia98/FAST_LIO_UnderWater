#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;

class Preprocess
{
public:
  void process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
               PointCloudXYZI::Ptr &pcl_out);

  double blind = 0.01;

private:
  void default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  PointCloudXYZI pl_surf;
};

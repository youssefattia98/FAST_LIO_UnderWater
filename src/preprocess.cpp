#include "preprocess.h"

#include <cstring>
#include <limits>

void Preprocess::process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
                         PointCloudXYZI::Ptr &pcl_out)
{
  default_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();

  const auto point_count = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  if (point_count == 0 || msg->point_step == 0 || msg->data.empty())
    return;

  const sensor_msgs::msg::PointField *x_field = nullptr;
  const sensor_msgs::msg::PointField *y_field = nullptr;
  const sensor_msgs::msg::PointField *z_field = nullptr;
  const sensor_msgs::msg::PointField *intensity_field = nullptr;

  for (const auto &field : msg->fields)
  {
    if (field.name == "x") x_field = &field;
    else if (field.name == "y") y_field = &field;
    else if (field.name == "z") z_field = &field;
    else if (field.name == "intensity") intensity_field = &field;
  }

  if (x_field == nullptr || y_field == nullptr || z_field == nullptr)
    return;

  auto read_field_as_float = [](const uint8_t *base, const sensor_msgs::msg::PointField *field) -> float
  {
    const uint8_t *ptr = base + field->offset;
    switch (field->datatype)
    {
      case sensor_msgs::msg::PointField::INT8:
      {
        int8_t value;
        std::memcpy(&value, ptr, sizeof(value));
        return static_cast<float>(value);
      }
      case sensor_msgs::msg::PointField::UINT8:
      {
        uint8_t value;
        std::memcpy(&value, ptr, sizeof(value));
        return static_cast<float>(value);
      }
      case sensor_msgs::msg::PointField::INT16:
      {
        int16_t value;
        std::memcpy(&value, ptr, sizeof(value));
        return static_cast<float>(value);
      }
      case sensor_msgs::msg::PointField::UINT16:
      {
        uint16_t value;
        std::memcpy(&value, ptr, sizeof(value));
        return static_cast<float>(value);
      }
      case sensor_msgs::msg::PointField::INT32:
      {
        int32_t value;
        std::memcpy(&value, ptr, sizeof(value));
        return static_cast<float>(value);
      }
      case sensor_msgs::msg::PointField::UINT32:
      {
        uint32_t value;
        std::memcpy(&value, ptr, sizeof(value));
        return static_cast<float>(value);
      }
      case sensor_msgs::msg::PointField::FLOAT64:
      {
        double value;
        std::memcpy(&value, ptr, sizeof(value));
        return static_cast<float>(value);
      }
      case sensor_msgs::msg::PointField::FLOAT32:
      default:
      {
        float value;
        std::memcpy(&value, ptr, sizeof(value));
        return value;
      }
    }
  };

  const bool intensity_missing = (intensity_field == nullptr);
  const float fallback_intensity = std::numeric_limits<float>::max();

  pl_surf.reserve(point_count);

  for (size_t row = 0; row < msg->height; ++row)
  {
    const size_t row_offset = row * msg->row_step;
    for (size_t col = 0; col < msg->width; ++col)
    {
      const uint8_t *point_ptr = msg->data.data() + row_offset + col * msg->point_step;

      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = read_field_as_float(point_ptr, x_field);
      added_pt.y = read_field_as_float(point_ptr, y_field);
      added_pt.z = read_field_as_float(point_ptr, z_field);
      added_pt.intensity = intensity_missing ? fallback_intensity : read_field_as_float(point_ptr, intensity_field);
      added_pt.curvature = 0.;

      const double range2 = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
      if (range2 > (blind * blind))
      {
        pl_surf.push_back(std::move(added_pt));
      }
    }
  }
}

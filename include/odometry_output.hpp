#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "odometry_covariance.hpp"

namespace uwfl2
{

template <typename State, typename Covariance>
nav_msgs::msg::Odometry make_odometry(
    const State &state, const Covariance &covariance,
    const builtin_interfaces::msg::Time &stamp,
    const geometry_msgs::msg::Quaternion &orientation)
{
    nav_msgs::msg::Odometry output;
    output.header.frame_id = "camera_init";
    output.child_frame_id = "body";
    output.header.stamp = stamp;
    output.pose.pose.position.x = state.pos.x();
    output.pose.pose.position.y = state.pos.y();
    output.pose.pose.position.z = state.pos.z();
    output.pose.pose.orientation = orientation;
    output.pose.covariance = make_ros_pose_covariance(
        covariance, state.rot.toRotationMatrix()).values;
    return output;
}

inline geometry_msgs::msg::TransformStamped make_odometry_transform(
    const nav_msgs::msg::Odometry &odometry)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = odometry.child_frame_id;
    transform.transform.translation.x = odometry.pose.pose.position.x;
    transform.transform.translation.y = odometry.pose.pose.position.y;
    transform.transform.translation.z = odometry.pose.pose.position.z;
    transform.transform.rotation = odometry.pose.pose.orientation;
    return transform;
}

}  // namespace uwfl2

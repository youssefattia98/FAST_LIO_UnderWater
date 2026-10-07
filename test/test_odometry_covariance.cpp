#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "odometry_covariance.hpp"
#include "odometry_output.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

namespace
{

using Covariance27 = Eigen::Matrix<double, 27, 27>;

Eigen::Matrix<double, 6, 6> as_matrix(
    const std::array<double, 36> &values)
{
    Eigen::Matrix<double, 6, 6> result;
    for (int row = 0; row < 6; ++row)
    {
        for (int column = 0; column < 6; ++column)
        {
            result(row, column) = values[static_cast<std::size_t>(row * 6 + column)];
        }
    }
    return result;
}

TEST(OdometryCovariance, PublishesRosPositionThenOrientationOrder)
{
    Covariance27 covariance = Covariance27::Zero();
    covariance.diagonal().setConstant(1.0);
    covariance.block<3, 3>(0, 0).diagonal() << 1.0, 2.0, 3.0;
    covariance.block<3, 3>(3, 3).diagonal() << 4.0, 5.0, 6.0;

    const auto output =
        uwfl2::make_ros_pose_covariance(covariance, Eigen::Matrix3d::Identity());
    ASSERT_TRUE(output.valid);
    const auto ros = as_matrix(output.values);
    EXPECT_DOUBLE_EQ(ros(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(ros(2, 2), 3.0);
    EXPECT_DOUBLE_EQ(ros(3, 3), 4.0);
    EXPECT_DOUBLE_EQ(ros(5, 5), 6.0);
}

TEST(OdometryCovariance, RotatesRightAttitudeErrorIntoOdometryFrame)
{
    Covariance27 covariance = Covariance27::Identity();
    covariance.block<3, 3>(3, 3).setZero();
    covariance(3, 3) = 1.0;
    covariance(4, 4) = 4.0;
    covariance(5, 5) = 9.0;
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    const auto output = uwfl2::make_ros_pose_covariance(covariance, rotation);
    ASSERT_TRUE(output.valid);
    const auto ros = as_matrix(output.values);
    EXPECT_NEAR(ros(3, 3), 4.0, 1e-12);
    EXPECT_NEAR(ros(4, 4), 1.0, 1e-12);
    EXPECT_NEAR(ros(5, 5), 9.0, 1e-12);
}

TEST(OdometryCovariance, RepairsSmallNegativeEigenvalueForRviz)
{
    Covariance27 covariance = Covariance27::Identity();
    covariance(0, 0) = -1e-12;

    const auto output =
        uwfl2::make_ros_pose_covariance(covariance, Eigen::Matrix3d::Identity());
    ASSERT_TRUE(output.valid);
    EXPECT_TRUE(output.projected_to_psd);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
        as_matrix(output.values));
    ASSERT_EQ(solver.info(), Eigen::Success);
    EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-15);
}

}  // namespace

TEST(OdometryOutput, PreservesLegacySerializedFieldsAndQuaternionSigns)
{
    struct State
    {
        Eigen::Vector3d pos{1.0, -2.0, 3.0};
        Eigen::Quaterniond rot{Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized())};
    } state;
    const Covariance27 covariance = Covariance27::Identity();
    for (const double sign : {-1.0, 1.0})
    {
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = sign * state.rot.x();
        orientation.y = sign * state.rot.y();
        orientation.z = sign * state.rot.z();
        orientation.w = sign * state.rot.w();
        builtin_interfaces::msg::Time stamp;
        stamp.sec = 1780000000;
        stamp.nanosec = 123456789;
        nav_msgs::msg::Odometry legacy;
        legacy.header.frame_id = "camera_init";
        legacy.child_frame_id = "body";
        legacy.header.stamp = stamp;
        legacy.pose.pose.position.x = state.pos.x();
        legacy.pose.pose.position.y = state.pos.y();
        legacy.pose.pose.position.z = state.pos.z();
        legacy.pose.pose.orientation = orientation;
        legacy.pose.covariance = uwfl2::make_ros_pose_covariance(
            covariance, state.rot.toRotationMatrix()).values;
        const auto output = uwfl2::make_odometry(state, covariance, stamp, orientation);
        EXPECT_EQ(output, legacy);
        rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;
        rclcpp::SerializedMessage before, after;
        serializer.serialize_message(&legacy, &before);
        serializer.serialize_message(&output, &after);
        const auto &a = before.get_rcl_serialized_message();
        const auto &b = after.get_rcl_serialized_message();
        ASSERT_EQ(a.buffer_length, b.buffer_length);
        EXPECT_TRUE(std::equal(a.buffer, a.buffer + a.buffer_length, b.buffer));
        const auto transform = uwfl2::make_odometry_transform(output);
        EXPECT_EQ(transform.header, legacy.header);
        EXPECT_EQ(transform.child_frame_id, legacy.child_frame_id);
        EXPECT_EQ(transform.transform.rotation, orientation);
        EXPECT_DOUBLE_EQ(transform.transform.translation.x, state.pos.x());
        EXPECT_DOUBLE_EQ(transform.transform.translation.y, state.pos.y());
        EXPECT_DOUBLE_EQ(transform.transform.translation.z, state.pos.z());
    }
}

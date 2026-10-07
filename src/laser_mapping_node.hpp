#pragma once

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

std::shared_ptr<rclcpp::Node> MakeLaserMappingNode(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

void SigHandle(int sig);

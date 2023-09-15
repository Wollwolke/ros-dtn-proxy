#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/serialization.hpp>
#include <string>
#include <vector>

#include "pipeline/action_interface.hpp"
#include "pipeline/pipeline_msg.hpp"

namespace dtnproxy::pipeline {

class CombineTfAction : public IAction {
private:
    const uint SEQUENCE_NR_IN = 85;
    const uint SEQUENCE_NR_OUT = 10;
    const Direction dir = Direction::INOUT;

    std::string sourceFrame;
    std::string targetFrame;

    rclcpp::Serialization<geometry_msgs::msg::TransformStamped> tfSerialization;
    std::unique_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    static void appendMessage(std::vector<uint8_t>& buffer, rclcpp::SerializedMessage& msg,
                              bool empty = false);
    void combine(PipelineMessage& pMsg);
    void split(PipelineMessage& pMsg);

public:
    CombineTfAction(std::vector<std::string> params, rclcpp::Node& nodeHandle);

    Direction direction() override;
    uint order(const Direction& pipelineDir) override;
    bool run(PipelineMessage& pMsg, const Direction& pipelineDir) override;
};

}  // namespace dtnproxy::pipeline

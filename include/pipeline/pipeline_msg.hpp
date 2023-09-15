#pragma once

#include <memory>
#include <rclcpp/serialized_message.hpp>

namespace dtnproxy::pipeline {

using PipelineMessage = struct PipelineMessage {
    // Payload
    std::shared_ptr<rclcpp::SerializedMessage> serializedMessage;
    // Overwrite lifetime set in config file
    uint64_t lifetime = 0;
    // Flag to mark all older bundles on the same topic as expired
    bool markExpired = false;
};

}  // namespace dtnproxy::pipeline

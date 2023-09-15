#include "pipeline/combine_tf.hpp"

#include <netinet/in.h>
#include <tf2_ros/buffer.h>

#include <cstdint>
#include <cstring>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/serialized_message.hpp>

namespace dtnproxy::pipeline {

void CombineTfAction::appendMessage(std::vector<uint8_t>& buffer, rclcpp::SerializedMessage& msg,
                                    bool empty) {
    uint32_t msgLength = 0;
    if (!empty) {
        msgLength = htonl(static_cast<uint32_t>(msg.get_rcl_serialized_message().buffer_length));
    }

    // append serialized msg length
    auto* msgLengthPtr = reinterpret_cast<uint8_t*>(&msgLength);
    buffer.insert(buffer.end(), msgLengthPtr, msgLengthPtr + sizeof(msgLength));

    if (!empty) {
        // append msg
        auto cdrMsg = msg.get_rcl_serialized_message();
        buffer.insert(buffer.end(), cdrMsg.buffer, cdrMsg.buffer + cdrMsg.buffer_length);
    }
}

void CombineTfAction::combine(PipelineMessage& pMsg) {
    std::vector<uint8_t> buffer;
    rclcpp::SerializedMessage serializedTf;

    // Append TF Message
    if (sourceFrame.empty() || targetFrame.empty()) {
        std::cout << "CombineTF: ⚠ No Transform added - Check sourceFrame / targetFrame parameter"
                  << std::endl;
        appendMessage(buffer, serializedTf, true);
    } else {
        try {
            // TODO: use header timestamp, if available
            geometry_msgs::msg::TransformStamped transform =
                tfBuffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);
            tfSerialization.serialize_message(&transform, &serializedTf);
            appendMessage(buffer, serializedTf);

        } catch (const tf2::TransformException& ex) {
            std::cout << "CombineTF: ⚠ No Transform added - Could not transform '" << sourceFrame
                      << "' to '" << targetFrame << "': " << ex.what() << std::endl;
            appendMessage(buffer, serializedTf, true);
        }
    }

    // Append Topic Message
    auto* cdrMsg = &pMsg.serializedMessage->get_rcl_serialized_message();
    buffer.insert(buffer.end(), cdrMsg->buffer, cdrMsg->buffer + cdrMsg->buffer_length);

    pMsg.serializedMessage->reserve(buffer.size());
    cdrMsg = &pMsg.serializedMessage->get_rcl_serialized_message();
    std::memcpy(cdrMsg->buffer, &buffer.front(), buffer.size());
    cdrMsg->buffer_length = buffer.size();
}

void CombineTfAction::split(PipelineMessage& pMsg) {
    auto cdrMsg = pMsg.serializedMessage->get_rcl_serialized_message();

    uint32_t msgLength;
    const auto SIZE_OF_LEN = sizeof(msgLength);
    size_t offset = 0;

    // get TF Message length
    std::memcpy(&msgLength, cdrMsg.buffer + offset, SIZE_OF_LEN);
    msgLength = ntohl(msgLength);
    offset += SIZE_OF_LEN;

    if (0 != msgLength) {
        // send TF Message
        rclcpp::SerializedMessage serializedTf(msgLength);
        auto* newMsg = &serializedTf.get_rcl_serialized_message();
        newMsg->buffer_length = msgLength;
        std::memcpy(newMsg->buffer, cdrMsg.buffer + offset, msgLength);
        geometry_msgs::msg::TransformStamped transform;
        tfSerialization.deserialize_message(&serializedTf, &transform);
        tfBroadcaster->sendTransform(transform);

        offset += msgLength;
    }

    // forward Topic Message
    msgLength = cdrMsg.buffer_length - offset;
    pMsg.serializedMessage->reserve(msgLength);
    auto* newCdrMsg = &pMsg.serializedMessage->get_rcl_serialized_message();
    newCdrMsg->buffer_length = msgLength;
    std::memcpy(newCdrMsg->buffer, cdrMsg.buffer + offset, msgLength);
}

CombineTfAction::CombineTfAction(std::vector<std::string> params, rclcpp::Node& nodeHandle) {
    if (params.size() != 2) {
        std::cout << "CombineTF: 💥 Exactly two parameters required - sourceFrameId, targetFrameId"
                  << std::endl;
    } else {
        sourceFrame = params[0];
        targetFrame = params[1];
    }

    tfBuffer = std::make_unique<tf2_ros::Buffer>(nodeHandle.get_clock());
    tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
    tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(nodeHandle);
}

Direction CombineTfAction::direction() { return dir; }

uint CombineTfAction::order(const Direction& pipelineDir) {
    switch (pipelineDir) {
        case Direction::IN:
            return SEQUENCE_NR_IN;
            break;
        case Direction::OUT:
            return SEQUENCE_NR_OUT;
            break;
        case Direction::INOUT:
        default:
            throw new std::runtime_error("Pipeline should not have Direction INOUT");
    }
}

bool CombineTfAction::run(PipelineMessage& pMsg, const Direction& pipelineDir) {
    switch (pipelineDir) {
        case Direction::IN:
            combine(pMsg);
            break;
        case Direction::OUT:
            split(pMsg);
            break;
        case Direction::INOUT:
        default:
            throw new std::runtime_error("Pipeline should not have Direction INOUT");
    }
    return true;
}

}  // namespace dtnproxy::pipeline

#include "ros/manager_base.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace dtnproxy::ros {

ManagerBase::ManagerBase(rclcpp::Node& nodeHandle, conf::RosConfig config,
                         std::shared_ptr<DtndClient> dtn, const std::unique_ptr<Logger>& log)
    : nodeHandle(nodeHandle), log(log), dtn(dtn), config(config) {}

void ManagerBase::setStatsRecorder(std::shared_ptr<StatsRecorder> statsRecorder) {
    stats = statsRecorder;
}

uint32_t ManagerBase::getRosMsgSize(const std::shared_ptr<rclcpp::SerializedMessage>& msg) const {
    auto rosBufferSize = static_cast<uint32_t>(msg->get_rcl_serialized_message().buffer_length);
    return rosBufferSize + CDR_MSG_SIZE_OFFSET;
}

uint32_t ManagerBase::buildDtnPayload(std::vector<uint8_t>& payload,
                                      const std::shared_ptr<rclcpp::SerializedMessage>& msg,
                                      int reqId) {
    // Payload layout
    // [RequestHeaderId [1 Byte]] - Payload [X Bytes]

    auto cdrMsg = msg->get_rcl_serialized_message();
    auto rosBufferSize = static_cast<uint32_t>(cdrMsg.buffer_length);

    if (-1 != reqId) {
        payload.push_back(static_cast<uint8_t>(reqId));
    }

    payload.insert(payload.end(), cdrMsg.buffer, cdrMsg.buffer + cdrMsg.buffer_length);
    return rosBufferSize + CDR_MSG_SIZE_OFFSET;
}

std::string ManagerBase::prefixTopic(const std::string& topicName, const std::string& nodeName,
                                     bool isService) {
    constexpr auto TOPIC_PREFIX = "/";

    auto expandedTopic = topicName;
    if ('/' == expandedTopic[0]) expandedTopic.erase(expandedTopic.begin());
    return rclcpp::expand_topic_or_service_name(expandedTopic, nodeHandle.get_name(),
                                                TOPIC_PREFIX + nodeName, isService);
}

}  // namespace dtnproxy::ros

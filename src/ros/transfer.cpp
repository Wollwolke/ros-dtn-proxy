#include "ros/transfer.hpp"

#include <arpa/inet.h>

#include <cstring>
#include <memory>
#include <vector>

#include "common.hpp"
#include "ros/dtn_msg_type.hpp"

namespace dtnproxy::ros {

Transfer::Transfer(rclcpp::Node& nodeHandle, conf::RosConfig config,
                   std::shared_ptr<DtndClient> dtn)
    : topics(nodeHandle, config, dtn, log), services(nodeHandle, config, dtn, log) {
    log = std::make_unique<Logger>("ros");
}

std::pair<std::string, ros::DtnMsgType> Transfer::splitEndpointAndType(
    const std::string& typedEndpoint) {
    using common::dtnPrefixes::INTERNAL;
    using common::dtnPrefixes::REQUEST;
    using common::dtnPrefixes::RESPONSE;
    using common::dtnPrefixes::TOPIC;

    // Topic
    if (typedEndpoint.rfind(TOPIC, 0) == 0) {
        return std::make_pair(typedEndpoint.substr(strlen(TOPIC)), DtnMsgType::TOPIC);
    }

    // Response
    if (typedEndpoint.rfind(RESPONSE, 0) == 0) {
        return std::make_pair(typedEndpoint.substr(strlen(RESPONSE)), DtnMsgType::RESPONSE);
    }

    // Request
    if (typedEndpoint.rfind(REQUEST, 0) == 0) {
        return std::make_pair(typedEndpoint.substr(strlen(REQUEST)), DtnMsgType::REQUEST);
    }

    // Internal
    if (typedEndpoint.rfind(INTERNAL, 0) == 0) {
        return std::make_pair(typedEndpoint.substr(strlen(INTERNAL)), DtnMsgType::INTERNAL);
    }

    return std::make_pair("", DtnMsgType::INVALID);
}

void Transfer::initSubscribersAndClients() {
    topics.initSubscriber();
    services.initClients();
}

void Transfer::enableStatsRecorder(std::shared_ptr<StatsRecorder> statsRecorder) {
    stats = statsRecorder;
    topics.setStatsRecorder(statsRecorder);
    services.setStatsRecorder(statsRecorder);
}

void Transfer::onDtnMessage(const data::WsReceive& bundle) {
    constexpr auto SIZE_OF_HEADER_ID = 1;

    auto data = bundle.data;
    auto typedEndpoint = bundle.dst;

    auto [topic, type] = splitEndpointAndType(typedEndpoint);
    // TODO: get msg type for stats
    if (stats) stats->dtnReceived("/" + bundle.src + topic, "unknown", data.size(), type);

    switch (type) {
        case DtnMsgType::TOPIC: {
            topics.onDtnMessage(topic, data, bundle.src);
        } break;
        case DtnMsgType::REQUEST: {
            std::vector<uint8_t> buffer(data.begin() + SIZE_OF_HEADER_ID, data.end());
            uint8_t headerId;
            memcpy(&headerId, &data.front(), SIZE_OF_HEADER_ID);
            services.onDtnRequest(topic, buffer, headerId, bundle.src);
        } break;
        case DtnMsgType::RESPONSE: {
            std::vector<uint8_t> buffer(data.begin() + SIZE_OF_HEADER_ID, data.end());
            uint8_t headerId;
            memcpy(&headerId, &data.front(), SIZE_OF_HEADER_ID);
            services.onDtnResponse(topic, buffer, headerId, bundle.src);
        } break;
        case DtnMsgType::INTERNAL: {
            topics.onInternalMsg(topic, data, bundle.src);
            services.onInternalMsg(topic, data, bundle.src);
        } break;
        case DtnMsgType::INVALID:
        default:
            // Not a message for us, ignoring...
            return;
    }
}

}  // namespace dtnproxy::ros

#include "ros/service_manager.hpp"

#include <arpa/inet.h>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <string>
#include <vector>

#include "ros/dtn_msg_type.hpp"

namespace dtnproxy::ros {

using SharedFuture = std::shared_future<std::shared_ptr<rclcpp::SerializedMessage>>;

int ServiceManager::requestHeaderId(rmw_request_id_t requestHeader) {
    if (storedRequestHeaders.size() >= MAX_WAITING_REQUESTS) {
        return -1;
    }
    storedRequestHeaders.push_back(requestHeader);
    return storedRequestHeaders.size() - 1;
}

void ServiceManager::responseCallback(const std::string& topic, const std::string& remoteNodeId,
                                      uint8_t requestId,
                                      std::shared_ptr<rclcpp::SerializedMessage> response) {
    std::vector<uint8_t> payload;
    auto rosMsgSize = buildDtnPayload(payload, response, requestId);
    if (stats) stats->rosReceived(topic, "unknown", rosMsgSize, DtnMsgType::RESPONSE);

    DtndClient::Message dtnMsg{
        std::move(payload),      // std::vector<uint8_t> payload
        topic,                   // std::string endpoint
        DtnMsgType::RESPONSE,    // ros::DtnMsgType msgType
        0,                       // uint64_t bundleFlags = 0
        0,                       // uint64_t lifetime = 0
        "dtn://" + remoteNodeId  // std::string remoteNodeId = ""
    };
    dtn->sendMessage(dtnMsg);
    if (stats) stats->dtnSent(topic, "unknown", dtnMsg.payload.size(), DtnMsgType::RESPONSE);
}

void ServiceManager::requestCallback(const std::string& topic, const std::string& type,
                                     const std::string& remoteNodeId,
                                     std::shared_ptr<rmw_request_id_t> requestHeader,
                                     std::shared_ptr<rclcpp::SerializedMessage> request) {
    // TODO: think about using the smart pointer here
    auto reqId = requestHeaderId(*requestHeader);
    if (-1 == reqId) {
        log->WARN() << "More than " << MAX_WAITING_REQUESTS << " open service requests. Ignoring.";
        return;
    }

    std::vector<uint8_t> payload;
    auto rosMsgSize = buildDtnPayload(payload, request, reqId);

    auto prefixedTopic = prefixTopic(topic, remoteNodeId, true);
    if (stats) stats->rosReceived(prefixedTopic, type, rosMsgSize, DtnMsgType::REQUEST);

    DtndClient::Message dtnMsg{
        std::move(payload),      // std::vector<uint8_t> payload
        topic,                   // std::string endpoint
        DtnMsgType::REQUEST,     // ros::DtnMsgType msgType
        0,                       // uint64_t bundleFlags = 0
        0,                       // uint64_t lifetime = 0
        "dtn://" + remoteNodeId  // std::string remoteNodeId = ""
    };
    dtn->sendMessage(dtnMsg);
    if (stats) stats->dtnSent(prefixedTopic, type, dtnMsg.payload.size(), DtnMsgType::REQUEST);
}

ServiceManager::ServiceManager(rclcpp::Node& nodeHandle, conf::RosConfig config,
                               std::shared_ptr<DtndClient> dtn, const std::unique_ptr<Logger>& log)
    : ManagerBase(nodeHandle, config, dtn, log) {}

void ServiceManager::onDtnRequest(const std::string& topic, std::vector<uint8_t>& data,
                                  uint8_t headerId, const std::string& src) {
    rcl_serialized_message_t request{
        &data.front(),               // buffer
        data.size(),                 // buffer_length
        data.size(),                 // buffer_capacity
        rcl_get_default_allocator()  // allocator
    };
    auto sharedRequest = std::make_shared<rclcpp::SerializedMessage>(request);

    auto responseReceivedCallback = [this, topic, headerId, src](SharedFuture future) {
        const auto responseMsg = future.get();
        responseCallback(topic, src, headerId, responseMsg);
    };

    using namespace std::chrono_literals;
    try {
        while (!clients.at(topic)->wait_for_service(5s)) {
            log->INFO() << "Service " << topic << " not available, waiting...";
        }
        clients.at(topic)->async_send_request(sharedRequest, responseReceivedCallback);
    } catch (const std::out_of_range& ex) {
        log->WARN() << "Ignoring message for service " << topic << " which is not configured!";
    }

    // TODO: find msgType in rosConfig
    if (stats) {
        stats->rosSent(topic, "unknown", static_cast<uint32_t>(data.size()) + CDR_MSG_SIZE_OFFSET,
                       DtnMsgType::REQUEST);
    }
}

void ServiceManager::onDtnResponse(const std::string& topic, std::vector<uint8_t>& data,
                                   uint8_t headerId, const std::string& src) {
    rcl_serialized_message_t response{
        &data.front(),               // buffer
        data.size(),                 // buffer_length
        data.size(),                 // buffer_capacity
        rcl_get_default_allocator()  // allocator
    };
    auto sharedResponse = std::make_shared<rclcpp::SerializedMessage>(response);

    auto requestHeader = storedRequestHeaders.at(headerId);
    storedRequestHeaders.erase(storedRequestHeaders.begin() + headerId);

    auto prefixedTopic = prefixTopic(topic, src, true);
    servers.at(prefixedTopic)->send_response(requestHeader, sharedResponse);

    // TODO: find msgType in rosConfig
    if (stats) {
        stats->rosSent(prefixedTopic, "unknown",
                       static_cast<uint32_t>(data.size()) + CDR_MSG_SIZE_OFFSET,
                       DtnMsgType::RESPONSE);
    }
}

void ServiceManager::onInternalMsg(const std::string& endpoint, std::vector<uint8_t>& data,
                                   const std::string& src) {
    constexpr auto TOPIC_PREFIX = "/dtn_proxy";

    if (common::REMOTE_CONFIG_ENDPOINT == endpoint) {
        auto serviceOptions = rcl_service_get_default_options();
        auto jConfig = nlohmann::json::from_cbor(data);
        auto remoteConfig = jConfig.get<conf::RemoteConfig>();
        for (auto const& interface : remoteConfig.interfaces) {
            if (!interface.isService) {
                continue;
            }

            auto prefixedTopic = prefixTopic(interface.topic, src, true);
            auto cb = std::bind(&ServiceManager::requestCallback, this, interface.topic,
                                interface.type, src, std::placeholders::_1, std::placeholders::_2);
            auto server = GenericService::make_shared(
                nodeHandle.get_node_base_interface()->get_shared_rcl_node_handle(),
                TOPIC_PREFIX + prefixedTopic, interface.type, cb, serviceOptions);

            // [nullptr] uses default mutually exclusive callback group
            nodeHandle.get_node_services_interface()->add_service(server, nullptr);
            servers.insert_or_assign(prefixedTopic, server);

            log->INFO() << "Providing service:\t/dtn_proxy" << prefixedTopic;
        }
    }
}

void ServiceManager::initClients() {
    auto clientOptions = rcl_client_get_default_options();
    clients.clear();

    for (const auto& [topic, type, profile] : config.clients) {
        auto client = GenericClient::make_shared(nodeHandle.get_node_base_interface().get(),
                                                 nodeHandle.get_node_graph_interface(), topic, type,
                                                 clientOptions);

        // [nullptr] uses default mutually exclusive callback group
        nodeHandle.get_node_services_interface()->add_client(client, nullptr);
        clients.insert_or_assign(topic, client);

        using namespace std::chrono_literals;
        if (!client->wait_for_service(1s)) {
            log->WARN() << "Service " << topic << " not available!";
        }
    }
}

}  // namespace dtnproxy::ros

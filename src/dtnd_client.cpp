#include "dtnd_client.hpp"

#include <nlohmann/json.hpp>

#include "common.hpp"

namespace dtnproxy {

DtndClient::DtndClient(const conf::DtnConfig& config) : config(config) {
    messageHandler = [](const data::WsReceive&) {};
    endpointsToRegister.insert(
        buildEndpointId(common::REMOTE_CONFIG_ENDPOINT, ros::DtnMsgType::INTERNAL));

    log = std::make_unique<Logger>("dtn");
    http = std::make_unique<httplib::Client>(config.address, config.port);
    ws = std::make_unique<WsClient>();

    ws->setConnectionStatusHandler(
        std::bind(&DtndClient::onConnectionStatus, this, std::placeholders::_1));
    ws->setBundleHandler(std::bind(&DtndClient::onBundle, this, std::placeholders::_1));
    ws->connect("ws://" + config.address + ":" + std::to_string(config.port) + "/ws");
}

void DtndClient::setMessageHandler(messageHandler_t h) { messageHandler = h; }

void DtndClient::onConnectionStatus(const bool success) {
    std::unique_lock<std::mutex> lock(stateMutex);
    if (success) {
        wsStatus = WsState::CONNECTED;
        lock.unlock();
        log->INFO() << "WS Connection to dtnd opened.";
        registerKnownEndpoints();
        sendRemoteConfig();
    } else {
        wsStatus = WsState::ERROR;
        log->WARN() << "WS Connection to dtnd failed.";
    }
}

void DtndClient::onBundle(const std::string& bundle) {
    nlohmann::json j = nlohmann::json::from_cbor(bundle);
    log->DBG() << j;

    auto wsBundle = j.get<data::WsReceive>();
    removeNodeIdFromEndpoint(wsBundle.dst);
    removeProtocolFromNodeId(wsBundle.src);

    messageHandler(wsBundle);

    // register dnt endpoints after ros is initialized through messageHandler
    if (wsBundle.dst.rfind(common::dtnPrefixes::INTERNAL, 0) == 0) {
        onInternalMsg(wsBundle);
    }
}

DtndClient::Result::Result(bool success, std::string content)
    : success(success), content(content) {}

bool DtndClient::registerSubscribeEndpoints() {
    std::lock_guard<std::mutex> lock(endpointsMutex);

    for (auto& endpointId : endpointsToRegister) {
        auto result = getRequest("/register?" + endpointId);
        log->DBG() << result.content;
        if (!result.success) return false;

        ws->send("/subscribe " + endpointId);
    }
    return true;
}

std::string DtndClient::buildEndpointId(const std::string& endpoint, ros::DtnMsgType type) {
    using namespace common::dtnPrefixes;
    using Type = ros::DtnMsgType;

    auto result = endpoint;
    switch (type) {
        case Type::TOPIC:
            result.insert(0, TOPIC);
            break;
        case Type::REQUEST:
            result.insert(0, REQUEST);
            break;
        case Type::RESPONSE:
            result.insert(0, RESPONSE);
            break;
        case Type::INTERNAL:
            result.insert(0, INTERNAL);
            break;
        case Type::INVALID:
        default:
            break;
    }
    return result;
}

void DtndClient::registerKnownEndpoints() {
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        if (WsState::CONNECTED != wsStatus) {
            log->ERR() << "Can't register endpionts, not connection to dtnd!";
            return;
        }
    }
    // TODO: split this method
    // Set WS to cbor data mode
    ws->send("/data");

    bool ok = true;
    ok &= getLocalNodeId();
    ok &= registerSubscribeEndpoints();
    if (!ok) {
        log->FATAL() << "Error registering endopints!";
    }
}

void DtndClient::removeNodeIdFromEndpoint(std::string& endpoint) {
    // remove first 6 chars (protocol)
    endpoint.erase(0, 6);
    auto stringPos = endpoint.find("/");
    endpoint.erase(0, stringPos + 1);
}

void DtndClient::removeProtocolFromNodeId(std::string& nodeId) {
    // remove first 6 chars (protocol) and ending '/'
    // dtn://node1/ -> node1
    nodeId.erase(0, 6);
    nodeId.erase(nodeId.size() - 1);
}

void DtndClient::onInternalMsg(data::WsReceive bundle) {
    if (bundle.dst.find(common::REMOTE_CONFIG_ENDPOINT) != std::string::npos) {
        log->INFO() << "Received Configuration message";

        auto jConfig = nlohmann::json::from_cbor(bundle.data);
        auto remoteConfig = jConfig.get<conf::RemoteConfig>();
        std::unique_lock<std::mutex> lock(endpointsMutex);
        for (auto const& interface : remoteConfig.interfaces) {
            auto endpoint = interface.topic;
            if ('/' != endpoint[0]) {
                endpoint.insert(0, "/");
            }
            if (interface.isService) {
                endpointsToRegister.insert(buildEndpointId(endpoint, ros::DtnMsgType::RESPONSE));
            } else {
                endpointsToRegister.insert(buildEndpointId(endpoint, ros::DtnMsgType::TOPIC));
            }
        }
        lock.unlock();
        registerKnownEndpoints();
    }
    // No other internal msgs implemented
}

void DtndClient::registerEndpoints(const std::vector<DtnEndpoint>& endpoints) {
    {
        std::lock_guard<std::mutex> lock(endpointsMutex);
        for (const auto& endpoint : endpoints) {
            endpointsToRegister.insert(buildEndpointId(endpoint.first, endpoint.second));
        }
    }
    registerKnownEndpoints();
}

void DtndClient::sendMessage(const Message& dtnMsg) {
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        if (WsState::CONNECTED != wsStatus) {
            log->ERR() << "Can't send message, not connection to dtnd!";
            return;
        }
    }

    const auto MS_IN_SECOND = 1000;

    auto typedEndpoint = buildEndpointId(dtnMsg.endpoint, dtnMsg.msgType);

    auto lifetime = (dtnMsg.lifetime == 0) ? config.lifetime : dtnMsg.lifetime;
    auto remoteNodeId = dtnMsg.remoteNodeId.empty() ? config.remoteNodeId : dtnMsg.remoteNodeId;

    data::WsSend msg{
        localNodeId,                         // std::string src,
        remoteNodeId + "/" + typedEndpoint,  // std::string dst,
        lifetime * MS_IN_SECOND,             // uint64_t lifetime,
        dtnMsg.bundleFlags,                  // uint64_t bundle_flags,
        dtnMsg.payload,                      // std::vector<uint8_t>& data,
    };
    std::vector<uint8_t> cborMsg = nlohmann::json::to_cbor(msg);
    ws->send(std::move(cborMsg));
}

void DtndClient::sendRemoteConfig(std::vector<uint8_t> config) {
    std::lock_guard<std::mutex> lock(remoteConfigMutex);
    if (!config.empty()) {
        latestRemoteConfig = std::move(config);
    }

    if (!latestRemoteConfig.empty()) {
        Message msg = {latestRemoteConfig, common::REMOTE_CONFIG_ENDPOINT,
                       ros::DtnMsgType::INTERNAL, BundleFlags::BUNDLE_REMOVE_OLDER_BUNDLES,
                       common::REMOTE_CONFIG_LIFETIME};
        sendMessage(msg);
    }
}

DtndClient::Result DtndClient::getRequest(std::string path) {
    using namespace std::literals::string_literals;
    std::string content = "";
    if (auto res = http->Get(path)) {
        if (res->status == 200) {
            return Result(true, res->body);
        } else {
            content = "HTTP return code: "s + std::to_string(res->status) + " - "s + res->reason;
        }
    } else {
        content = "HTTP Client Error: "s + to_string(res.error());
    }

    return Result(false, content);
}

bool DtndClient::getLocalNodeId() {
    auto result = getRequest("/status/nodeid");
    if (result.success) {
        localNodeId = result.content;
        return true;
    } else {
        log->ERR() << "Requesting nodeId: " << result.content;
        return false;
    }
}

}  // namespace dtnproxy

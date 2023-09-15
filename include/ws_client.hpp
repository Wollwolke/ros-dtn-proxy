#pragma once
#include <functional>
#include <memory>
#include <mutex>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

#include "logger.hpp"

namespace dtnproxy {

class WsClient {
private:
    static constexpr auto WS_PING_RATE = 5;
    static constexpr auto WS_PONG_TIMEOUT = 2000;
    static constexpr auto RECONNECT_WATE_TIME = 5;
    enum Status : uint8_t;

    using client = websocketpp::client<websocketpp::config::asio_client>;
    using bundleHandler_t = std::function<void(const std::string&)>;
    using connectionStatusHandler_t = std::function<void(bool)>;

    using Metadata = struct Metadata {
        websocketpp::connection_hdl hdl;
        Status status;
        std::string errorReason;
    };

    bundleHandler_t bundleHandler;
    connectionStatusHandler_t connectionStatusHandler;
    client endpoint;
    Metadata metadata;
    std::shared_ptr<websocketpp::lib::thread> thread;
    std::mutex pingLoopMutex;
    std::condition_variable pingLoopCv;

    std::unique_ptr<Logger> log;

    void pingLoop();
    void tryReconnect(const std::string& uri);

public:
    WsClient();
    ~WsClient();

    void setBundleHandler(bundleHandler_t h);
    void setConnectionStatusHandler(connectionStatusHandler_t h);

    bool connect(const std::string& uri);
    void close(websocketpp::close::status::value code);
    void send(const std::string& msg);
    void send(const std::vector<uint8_t>& msg);

    void onOpen(client* c, websocketpp::connection_hdl hdl);
    void onFail(client* c, websocketpp::connection_hdl hdl);
    void onClose(client* c, websocketpp::connection_hdl hdl);
    void onMessage(websocketpp::connection_hdl hdl, client::message_ptr msg);
    void onPong(websocketpp::connection_hdl hdl, std::string payload);
    void onPongTimeout(websocketpp::connection_hdl hdl, std::string payload);

    friend std::ostream& operator<<(std::ostream& out, const Metadata& data);
    friend std::ostream& operator<<(std::ostream& out, const Status& status);
};

enum WsClient::Status : uint8_t { UNKNOWN, CLOSED, CONNECTING, OPEN, FAILED };

}  // namespace dtnproxy

#include "ws_client.hpp"

#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

namespace dtnproxy {

void WsClient::pingLoop() {
    websocketpp::lib::error_code errorCode;
    auto timepoint = std::chrono::system_clock::now() + std::chrono::seconds(WS_PING_RATE);

    while (true) {
        std::unique_lock<std::mutex> lock(pingLoopMutex);
        auto reason = pingLoopCv.wait_until(lock, timepoint);

        // stop ping thread, when connection isn't open anymore
        if (Status::OPEN != metadata.status) {
            return;
        }

        if (std::cv_status::timeout == reason) {
            timepoint = std::chrono::system_clock::now() + std::chrono::seconds(WS_PING_RATE);
            endpoint.ping(metadata.hdl, "wolke", errorCode);

            if (errorCode) {
                log->ERR() << "Error sending WS Ping: " << errorCode.message();
            }
        }
    }
}

void WsClient::tryReconnect(const std::string& uri) {
    std::this_thread::sleep_for(std::chrono::seconds(RECONNECT_WATE_TIME));
    log->INFO() << "Trying to reconnect...";
    connect(uri);
}

WsClient::WsClient() {
    bundleHandler = [](const std::string&) {};
    connectionStatusHandler = [](bool) {};

    log = std::make_unique<Logger>("ws");

    metadata.status = Status::UNKNOWN;

    // Disable logging
    endpoint.clear_access_channels(websocketpp::log::alevel::all);
    endpoint.clear_error_channels(websocketpp::log::elevel::all);

    endpoint.init_asio();
    endpoint.start_perpetual();  // Waits for connections

    thread.reset(new std::thread(&client::run, &endpoint));
}

WsClient::~WsClient() {
    endpoint.stop_perpetual();

    close(websocketpp::close::status::going_away);

    thread->join();
}

void WsClient::setBundleHandler(bundleHandler_t h) { bundleHandler = h; }

void WsClient::setConnectionStatusHandler(connectionStatusHandler_t h) {
    connectionStatusHandler = h;
}

bool WsClient::connect(const std::string& uri) {
    websocketpp::lib::error_code errorCode;

    endpoint.reset();
    client::connection_ptr conReq = endpoint.get_connection(uri, errorCode);

    if (errorCode) {
        log->ERR() << "Connect initialization error: " << errorCode.message();
        return false;
    }

    metadata.hdl = conReq->get_handle();

    {
        std::lock_guard<std::mutex> lock(pingLoopMutex);
        metadata.status = Status::CONNECTING;
    }

    conReq->set_pong_timeout(WS_PONG_TIMEOUT);
    conReq->set_open_handler(std::bind(&WsClient::onOpen, this, &endpoint, std::placeholders::_1));
    conReq->set_fail_handler(std::bind(&WsClient::onFail, this, &endpoint, std::placeholders::_1));
    conReq->set_message_handler(
        std::bind(&WsClient::onMessage, this, std::placeholders::_1, std::placeholders::_2));
    conReq->set_close_handler(
        std::bind(&WsClient::onClose, this, &endpoint, std::placeholders::_1));
    conReq->set_pong_handler(
        std::bind(&WsClient::onPong, this, std::placeholders::_1, std::placeholders::_2));
    conReq->set_pong_timeout_handler(
        std::bind(&WsClient::onPongTimeout, this, std::placeholders::_1, std::placeholders::_2));

    endpoint.connect(conReq);

    return true;
}

void WsClient::close(websocketpp::close::status::value code) {
    std::lock_guard<std::mutex> lock(pingLoopMutex);

    if (metadata.status == Status::OPEN) {
        log->INFO() << "Closing Websocket connection.";

        websocketpp::lib::error_code errorCode;
        endpoint.close(metadata.hdl, code, "", errorCode);
        if (errorCode) {
            log->ERR() << "Error initiating close: " << errorCode.message();
            metadata.status = Status::UNKNOWN;
        } else {
            metadata.status = Status::CLOSED;
        }
        pingLoopCv.notify_all();
    }
}

void WsClient::send(const std::string& msg) {
    websocketpp::lib::error_code errorCode;

    endpoint.send(metadata.hdl, msg, websocketpp::frame::opcode::TEXT, errorCode);
    if (errorCode) {
        log->ERR() << "Error sending message: " << errorCode.message();
        return;
    }
}

void WsClient::send(const std::vector<uint8_t>& msg) {
    websocketpp::lib::error_code errorCode;

    endpoint.send(metadata.hdl, &msg.front(), msg.size(), websocketpp::frame::opcode::BINARY,
                  errorCode);
    if (errorCode) {
        log->ERR() << "Error sending binary message: " << errorCode.message();
        return;
    }
}

void WsClient::onOpen(client* /*c*/, websocketpp::connection_hdl /*hdl*/) {
    {
        std::lock_guard<std::mutex> lock(pingLoopMutex);
        metadata.status = Status::OPEN;
    }
    metadata.errorReason.clear();

    // start ping loop
    std::thread pingThread(&WsClient::pingLoop, this);
    pingThread.detach();

    connectionStatusHandler(true);
    log->INFO() << metadata;
}

void WsClient::onFail(client* c, websocketpp::connection_hdl hdl) {
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    {
        std::lock_guard<std::mutex> lock(pingLoopMutex);
        metadata.status = Status::FAILED;
    }
    pingLoopCv.notify_all();

    metadata.errorReason = con->get_ec().message();
    connectionStatusHandler(false);
    log->INFO() << metadata;

    tryReconnect(con->get_uri()->str());
}

void WsClient::onClose(client* c, websocketpp::connection_hdl hdl) {
    {
        std::lock_guard<std::mutex> lock(pingLoopMutex);
        metadata.status = Status::CLOSED;
    }
    pingLoopCv.notify_all();
    connectionStatusHandler(false);

    client::connection_ptr con = c->get_con_from_hdl(hdl);
    auto localCloseReason = con->get_local_close_code();
    std::stringstream s;
    s << "remote code: " << con->get_remote_close_code() << " ("
      << websocketpp::close::status::get_string(con->get_remote_close_code())
      << "), remote reason: " << con->get_remote_close_reason()
      << "\t-\t local code: " << localCloseReason << " ("
      << websocketpp::close::status::get_string(con->get_local_close_code())
      << "), local reason: " << con->get_local_close_reason();
    metadata.errorReason = s.str();
    log->INFO() << metadata;

    if (websocketpp::close::status::going_away != localCloseReason &&
        websocketpp::close::status::normal != localCloseReason) {
        tryReconnect(con->get_uri()->str());
    }
}

void WsClient::onMessage(websocketpp::connection_hdl /*hdl*/, client::message_ptr msg) {
    std::string payload = msg->get_payload();

    switch (msg->get_opcode()) {
        case websocketpp::frame::opcode::BINARY:
            bundleHandler(payload);
            log->DBG() << ">> " << websocketpp::utility::to_hex(payload);
            break;
        case websocketpp::frame::opcode::TEXT:
            // Response to command
            log->DBG() << ">> " << payload;
            break;
        default:
            log->WARN() << "Unexpected Opcode received: " << msg->get_opcode();
    }
}

void WsClient::onPong(websocketpp::connection_hdl /*hdl*/, std::string payload) {
    log->DBG() << "Received pong with payload: " << payload;
}

void WsClient::onPongTimeout(websocketpp::connection_hdl hdl, std::string payload) {
    log->WARN() << "Pong timeout with payload: " << payload;
    {
        std::lock_guard<std::mutex> lock(pingLoopMutex);
        metadata.status = Status::FAILED;
    }
    pingLoopCv.notify_all();
    connectionStatusHandler(false);

    auto con = endpoint.get_con_from_hdl(hdl);
    auto uri = con->get_uri()->str();

    tryReconnect(uri);
}

std::ostream& operator<<(std::ostream& out, const WsClient::Metadata& data) {
    out << "Status: " << data.status << "\t"
        << "Reason: " << (data.errorReason.empty() ? "N/A" : data.errorReason);
    return out;
}

std::ostream& operator<<(std::ostream& out, const WsClient::Status& status) {
    switch (status) {
        case WsClient::Status::UNKNOWN:
            return out << "Unknwon";
        case WsClient::Status::CLOSED:
            return out << "Closed";
        case WsClient::Status::CONNECTING:
            return out << "Connecting";
        case WsClient::Status::OPEN:
            return out << "Open";
        case WsClient::Status::FAILED:
            return out << "Failed";
            // omit default case to trigger compiler warning for missing cases
    };
    return out << static_cast<std::uint8_t>(status);
}

}  // namespace dtnproxy

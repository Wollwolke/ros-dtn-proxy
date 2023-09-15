#pragma once

#include <memory>
#include <rclcpp/serialized_message.hpp>
#include <string_view>
#include <vector>

#include "configuration.hpp"
#include "dtnd_client.hpp"
#include "stats_recorder.hpp"

namespace dtnproxy::ros {

class ManagerBase {
private:
    static constexpr auto BYTES_FOR_HASH = 16;

protected:
    struct CdrMsgHash {
        // !Only properly works with timestamped msgs!
        std::size_t operator()(const rcl_serialized_message_t& msg) const noexcept {
            const char* data = reinterpret_cast<const char*>(msg.buffer);
            std::string hashOver(data, BYTES_FOR_HASH);
            return std::hash<std::string>{}(hashOver);
            // return std::hash<std::string_view>{}(std::string_view(data, msg.buffer_length));
            // Hash over the whole msg doesn't work, because...

            //            seq ID?   seconds    nsecs     id len
            //           |-------| |-------| |-------| |-------|
            //           v       v v       v v       v v       v
            // 00000000: 0001 0000 2550 3d64 38f2 3b34 0500 0000  ....%P=d8.;4....
            // 00000010: 7465 7374 0000 0000 e001 0000 8002 0000  test............
            //           ^          ^^     ^ ^                 ^
            //           |----------||-----| |-----------------|
            //            frame id    ?????         data

            // ????? changes randomly / is not initialized?
        }
    };

    // Stats is using no overhead for DTN msgs, so ROS Overhead is also set to zero
    // const uint32_t CDR_MSG_SIZE_OFFSET = sizeof(size_t) + sizeof(size_t);
    const uint32_t CDR_MSG_SIZE_OFFSET = 0;

    rclcpp::Node& nodeHandle;
    const std::unique_ptr<Logger>& log;
    std::shared_ptr<StatsRecorder> stats;
    std::shared_ptr<DtndClient> dtn;
    conf::RosConfig config;

    ManagerBase(rclcpp::Node& nodeHandle, conf::RosConfig config, std::shared_ptr<DtndClient> dtn,
                const std::unique_ptr<Logger>& log);

    uint32_t getRosMsgSize(const std::shared_ptr<rclcpp::SerializedMessage>& msg) const;

    uint32_t buildDtnPayload(std::vector<uint8_t>& payload,
                             const std::shared_ptr<rclcpp::SerializedMessage>& msg, int reqId = -1);
    std::string prefixTopic(const std::string& topicName, const std::string& nodeName,
                            bool isService);

public:
    void setStatsRecorder(std::shared_ptr<StatsRecorder> statsRecorder);
};

}  // namespace dtnproxy::ros

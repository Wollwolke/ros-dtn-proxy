#pragma once

#include <cstddef>
#include <fstream>
#include <string>

#include "ros/dtn_msg_type.hpp"

namespace dtnproxy {

class StatsRecorder {
private:
    std::ofstream file;
    static std::string timestamp();

public:
    StatsRecorder(const std::string& statsDir);
    ~StatsRecorder();

    void rosReceived(const std::string& topic, const std::string& msgType, uint32_t size,
                     ros::DtnMsgType rosType, size_t msgHash = 0);
    void rosSent(const std::string& topic, const std::string& msgType, uint32_t size,
                 ros::DtnMsgType rosType, size_t msgHash = 0);
    void dtnReceived(const std::string& endpoint, const std::string& msgType, uint32_t size,
                     ros::DtnMsgType rosType);
    void dtnSent(const std::string& endpoint, const std::string& msgType, uint32_t size,
                 ros::DtnMsgType rosType);
};

}  // namespace dtnproxy

/*
Server:
- kriegt request über Ros
- verschickt request über Dtn
- kriegt response über Dtn
- verschickt response über Ros

Client:
- kriegt request über Dtn
- verschickt request über Ros
- kriegt response über Ros
- verschickt response über Dtn
*/

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include "configuration.hpp"
#include "dtnd_client.hpp"
#include "logger.hpp"
#include "ros/dtn_msg_type.hpp"
#include "ros/service_manager.hpp"
#include "ros/topic_manager.hpp"
#include "stats_recorder.hpp"
#include "ws_datatypes.hpp"

namespace dtnproxy::ros {

class Transfer {
private:
    using DtnEndpoint = std::pair<std::string, ros::DtnMsgType>;

    std::shared_ptr<StatsRecorder> stats;
    std::unique_ptr<Logger> log;

    TopicManager topics;
    ServiceManager services;

    static DtnEndpoint splitEndpointAndType(const std::string& typedEndpoint);

public:
    Transfer(rclcpp::Node& nodeHandle, conf::RosConfig config, std::shared_ptr<DtndClient> dtn);

    void initSubscribersAndClients();
    void enableStatsRecorder(std::shared_ptr<StatsRecorder> statsRecorder);
    void onDtnMessage(const data::WsReceive& bundle);
};

}  // namespace dtnproxy::ros

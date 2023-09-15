#pragma once

#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <string>
#include <utility>

#include "logger.hpp"
#include "pipeline/pipeline.hpp"
#include "ros/manager_base.hpp"

namespace dtnproxy::ros {
class TopicManager : public ManagerBase {
private:
    using subscriberPair =
        std::pair<std::shared_ptr<rclcpp::GenericSubscription>, pipeline::Pipeline>;
    using publisherPair = std::pair<std::shared_ptr<rclcpp::GenericPublisher>, pipeline::Pipeline>;

    std::map<std::string, subscriberPair> subscriber;
    std::map<std::string, publisherPair> publisher;

    void topicCallback(const std::string& topic, const std::string& type,
                       std::shared_ptr<rclcpp::SerializedMessage> msg);
    void dtnMsgCallback(const std::string& topic, std::shared_ptr<rclcpp::SerializedMessage> msg,
                        const std::string& src, bool skipPipeline = false);

public:
    TopicManager();
    TopicManager(rclcpp::Node& nodeHandle, conf::RosConfig config, std::shared_ptr<DtndClient> dtn,
                 const std::unique_ptr<Logger>& log);

    void onDtnMessage(const std::string& topic, std::vector<uint8_t>& data, const std::string& src);
    void onInternalMsg(const std::string& endpoint, std::vector<uint8_t>& data,
                       const std::string& src);
    void initSubscriber();
};

}  // namespace dtnproxy::ros

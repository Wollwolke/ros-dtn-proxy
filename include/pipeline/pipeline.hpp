#pragma once

#include <map>
#include <memory>
#include <rclcpp/serialized_message.hpp>
#include <vector>

#include "configuration.hpp"
#include "pipeline/action_interface.hpp"
#include "pipeline_msg.hpp"

namespace dtnproxy::pipeline {

class Pipeline {
private:
    using PipelineConfig = std::map<std::string, std::vector<conf::Module>>;
    using msgStorePtr_t = std::shared_ptr<std::map<std::string, PipelineMessage>>;
    using injectMsgCb_t = std::function<void(const std::string& topic,
                                             std::shared_ptr<rclcpp::SerializedMessage> msg)>;

    std::vector<std::unique_ptr<IAction>> actions;
    std::string msgType;
    std::string topic;
    msgStorePtr_t msgStore;
    injectMsgCb_t injectMsgCb;
    Direction direction;

    void build(const std::vector<conf::Module>& modules, rclcpp::Node& nodeHandle);

public:
    Pipeline(Direction dir, std::string msgType, std::string topic);

    void initPipeline(const PipelineConfig& config, const std::string& profile,
                      rclcpp::Node& nodeHandle);
    void initPipeline(const PipelineConfig& config, const std::string& profile,
                      rclcpp::Node& nodeHandle, msgStorePtr_t msgStore);
    void initPipeline(const PipelineConfig& config, const std::string& profile,
                      rclcpp::Node& nodeHandle, injectMsgCb_t injectMsgCb);
    void initPipeline(const std::vector<conf::Module>& modules, rclcpp::Node& nodeHandle,
                      injectMsgCb_t injectMsgCb);
    bool run(PipelineMessage& pMsg);
};

}  // namespace dtnproxy::pipeline

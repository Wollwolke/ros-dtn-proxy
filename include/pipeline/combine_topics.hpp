#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "pipeline/action_interface.hpp"
#include "pipeline/pipeline_msg.hpp"

namespace dtnproxy::pipeline {

class CombineTopicsAction : public IAction {
private:
    using msgStorePtr_t = std::shared_ptr<std::map<std::string, PipelineMessage>>;
    using injectMsgCb_t = std::function<void(const std::string& topic,
                                             std::shared_ptr<rclcpp::SerializedMessage> msg)>;

    const uint SEQUENCE_NR_IN = 95;
    const uint SEQUENCE_NR_OUT = 5;
    const Direction dir = Direction::INOUT;

    const std::vector<std::string> topics;
    const std::string currentTopic;
    msgStorePtr_t msgStore;
    injectMsgCb_t injectMsgCb;

    static void appendMessage(std::vector<uint8_t>& buffer,
                              std::shared_ptr<rclcpp::SerializedMessage> msg);
    bool combine(PipelineMessage& pMsg);
    bool split(PipelineMessage& pMsg);

public:
    CombineTopicsAction(std::string currentTopic, std::vector<std::string> topics,
                        msgStorePtr_t msgStore, injectMsgCb_t injectMsgCb);

    Direction direction() override;
    uint order(const Direction& pipelineDir) override;
    bool run(PipelineMessage& pMsg, const Direction& pipelineDir) override;
};

}  // namespace dtnproxy::pipeline

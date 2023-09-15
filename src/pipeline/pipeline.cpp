#include "pipeline/pipeline.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include "pipeline/combine_tf.hpp"
#include "pipeline/combine_topics.hpp"
#include "pipeline/expire_bundles.hpp"
#include "pipeline/image_processing.hpp"
#include "pipeline/lzmh.hpp"
#include "pipeline/modules.hpp"
#include "pipeline/on_change.hpp"
#include "pipeline/pipeline_msg.hpp"
#include "pipeline/rate_limit.hpp"

namespace dtnproxy::pipeline {

Pipeline::Pipeline(Direction dir, std::string msgType, std::string topic)
    : msgType(std::move(msgType)), topic(std::move(topic)), direction(dir) {}

void Pipeline::initPipeline(const PipelineConfig& config, const std::string& profile,
                            rclcpp::Node& nodeHandle) {
    if (!profile.empty()) {
        auto modules = config.at(profile);
        build(modules, nodeHandle);
    }
}

void Pipeline::initPipeline(const PipelineConfig& config, const std::string& profile,
                            rclcpp::Node& nodeHandle, msgStorePtr_t msgStore) {
    this->msgStore = std::move(msgStore);
    initPipeline(config, profile, nodeHandle);
}

void Pipeline::initPipeline(const PipelineConfig& config, const std::string& profile,
                            rclcpp::Node& nodeHandle, injectMsgCb_t injectMsgCb) {
    this->injectMsgCb = std::move(injectMsgCb);
    initPipeline(config, profile, nodeHandle);
}

void Pipeline::initPipeline(const std::vector<conf::Module>& modules, rclcpp::Node& nodeHandle,
                            injectMsgCb_t injectMsgCb) {
    if (!modules.empty()) {
        this->injectMsgCb = std::move(injectMsgCb);
        build(modules, nodeHandle);
    }
}

bool Pipeline::run(PipelineMessage& pMsg) {
    for (auto& action : actions) {
        if (!action->run(pMsg, direction)) {
            return false;
        }
    }
    return true;
}

void Pipeline::build(const std::vector<conf::Module>& modules, rclcpp::Node& nodeHandle) {
    for (const auto& [module, params] : modules) {
        std::unique_ptr<IAction> mod;
        switch (module) {
            case Module::RATE_LIMIT:
                mod = std::make_unique<RateLimitAction>(params);
                break;
            case Module::COMPRESS:
                mod = std::make_unique<LzmhAction>(msgType);
                break;
            case Module::IMG_COMPRESS:
                mod = std::make_unique<ImageProcessingAction>(msgType);
                break;
            case Module::EXPIRE:
                mod = std::make_unique<ExpireBundlesAction>();
                break;
            case Module::ON_CHANGE:
                mod = std::make_unique<OnChangeAction>(msgType);
                break;
            case Module::COMBINE:
                mod = std::make_unique<CombineTopicsAction>(topic, params, msgStore, injectMsgCb);
                break;
            case Module::COMBINE_TF:
                mod = std::make_unique<CombineTfAction>(params, nodeHandle);
                break;
        }
        // Check if module should run when msg enters / leaves proxy
        if ((mod->direction() & (this->direction | Direction::INOUT)) != 0) {
            actions.push_back(std::move(mod));
        }
    }

    std::sort(actions.begin(), actions.end(), [this](auto& first, auto& second) {
        return first->order(direction) < second->order(direction);
    });
}

}  // namespace dtnproxy::pipeline

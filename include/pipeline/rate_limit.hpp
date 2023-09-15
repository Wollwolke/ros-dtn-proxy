#pragma once

#include <chrono>
#include <vector>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class RateLimitAction : public IAction {
private:
    const uint SEQUENCE_NR_IN = 1;
    const int MS_IN_SECOND = 1000;
    unsigned int deltaT = 0;
    const Direction dir = Direction::IN;

    std::chrono::time_point<std::chrono::steady_clock> lastMsgSentTime;

public:
    RateLimitAction(std::vector<std::string> params);

    Direction direction() override;
    uint order(const Direction& pipelineDir) override;
    bool run(PipelineMessage& pMsg, const Direction& pipelineDir) override;
};

}  // namespace dtnproxy::pipeline

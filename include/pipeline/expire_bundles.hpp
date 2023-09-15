#pragma once

#include <chrono>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class ExpireBundlesAction : public IAction {
private:
    const Direction dir = Direction::IN;
    const uint SEQUENCE_NR_IN = 90;

public:
    Direction direction() override;
    uint order(const Direction& pipelineDir) override;
    bool run(PipelineMessage& pMsg, const Direction& pipelineDir) override;
};

}  // namespace dtnproxy::pipeline

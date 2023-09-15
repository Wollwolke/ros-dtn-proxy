#pragma once

#include <memory>
#include <rclcpp/serialized_message.hpp>

#include "pipeline/pipeline_msg.hpp"

namespace dtnproxy::pipeline {

enum Direction {
    IN = 1 << 0,
    OUT = 1 << 1,
    INOUT = 1 << 2,
};

class IAction {
private:
public:
    virtual ~IAction() = default;
    virtual bool run(PipelineMessage& pMsg, const Direction& pipelineDir) = 0;
    virtual Direction direction() = 0;
    virtual uint order(const Direction& pipelineDir) = 0;
};

}  // namespace dtnproxy::pipeline

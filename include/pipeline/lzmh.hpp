#pragma once

extern "C" {
#include "enc_dec.h"
#include "file_buffer.h"
}

#include <string>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class LzmhAction : public IAction {
private:
    static constexpr auto unSupportedMsgType = "sensor_msgs/msg/Image";
    static constexpr auto BITS_IN_BYTE = 8;
    const uint SEQUENCE_NR_IN = 100;
    const uint SEQUENCE_NR_OUT = 1;
    const Direction dir = Direction::INOUT;

    bool active = true;
    file_buffer_t* fbIn;
    file_buffer_t* fbOut;
    options_t options;

public:
    LzmhAction(const std::string& msgType);
    ~LzmhAction();
    LzmhAction(const LzmhAction&) = delete;
    LzmhAction& operator=(const LzmhAction&) = delete;

    Direction direction() override;
    uint order(const Direction& pipelineDir) override;
    bool run(PipelineMessage& pMsg, const Direction& pipelineDir) override;
};

}  // namespace dtnproxy::pipeline

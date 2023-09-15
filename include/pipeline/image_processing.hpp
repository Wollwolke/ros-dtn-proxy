#pragma once

#include <cstdint>
#include <map>
#include <rclcpp/serialized_message.hpp>
#include <string>
#include <tuple>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class ImageProcessingAction : public IAction {
private:
    static constexpr auto supportedMsgType = "sensor_msgs/msg/Image";
    static constexpr auto BIT_DEPTH = 8;
    static constexpr auto QOI_ENCODING_NO_ALPHA = 3;
    static constexpr auto QOI_ENCODING_ALPHA = 4;
    static constexpr std::array<uint8_t, 8> QOI_END_TAG = {0x00, 0x00, 0x00, 0x00,
                                                           0x00, 0x00, 0x00, 0x01};
    static constexpr auto ENCODING_OFFSET = 1;

    const uint SEQUENCE_NR_IN = 80;
    const uint SEQUENCE_NR_OUT = 15;
    const Direction dir = Direction::INOUT;

    const std::map<std::string, uint8_t> encodingMap = {
        {"rgb8", 0}, {"rgba8", 1}, {"bgr8", 2}, {"bgra8", 3}};
    bool active = true;

    bool compress(PipelineMessage& pMsg);
    bool decompress(PipelineMessage& pMsg);
    static std::tuple<bool, uint8_t> isEncodingSupported(const std::string& encoding);
    std::string getEncodingFromId(uint8_t encodingId);

public:
    ImageProcessingAction(const std::string& msgType);

    Direction direction() override;
    uint order(const Direction& pipelineDir) override;
    bool run(PipelineMessage& pMsg, const Direction& pipelineDir) override;
};

}  // namespace dtnproxy::pipeline

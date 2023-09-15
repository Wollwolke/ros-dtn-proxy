#pragma once

#include <cstdint>
#include <memory>
#include <ros2_babel_fish/babel_fish.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <string>
#include <vector>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class OnChangeAction : public IAction {
private:
    static constexpr auto DEFAULT_HEADER_OFFSET = 12;
    // to skip the fist three segments and compare header.frameId + data
    //              ???     seconds    nsecs     id len
    //           |-------| |-------| |-------| |-------|
    //           v       v v       v v       v v       v
    // 00000000: 0001 0000 2550 3d64 38f2 3b34 0500 0000

    const uint SEQUENCE_NR_IN = 5;
    const Direction dir = Direction::IN;

    bool active = true;
    uint8_t headerOffset = 0;
    std::vector<uint8_t> oldMsg;

    ros2_babel_fish::MessageTypeSupport::ConstSharedPtr msgTypeSupport;
    std::unique_ptr<ros2_babel_fish::MessageMembersIntrospection> msgMemberIntro;
    const rosidl_typesupport_introspection_cpp::MessageMembers* msgMembers;

public:
    OnChangeAction(std::string msgType);

    Direction direction() override;
    uint order(const Direction& pipelineDir) override;
    bool run(PipelineMessage& pMsg, const Direction& pipelineDir) override;
};

}  // namespace dtnproxy::pipeline

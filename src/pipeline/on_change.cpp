#include "pipeline/on_change.hpp"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <memory>
#include <rclcpp/serialization.hpp>
#include <string>
#include <utility>

#include "ros/type_helper.hpp"

namespace dtnproxy::pipeline {

OnChangeAction::OnChangeAction(std::string msgType) {
    using ros2_babel_fish::MessageMembersIntrospection;
    using rosidl_typesupport_introspection_cpp::MessageMembers;

    // init typesupport
    msgTypeSupport = ros2_babel_fish::BabelFish().get_message_type_support(msgType);
    msgMembers =
        static_cast<const MessageMembers *>(msgTypeSupport->introspection_type_support_handle.data);
    msgMemberIntro = std::make_unique<MessageMembersIntrospection>(
        MessageMembersIntrospection(msgMembers, msgTypeSupport->type_support_library));
}

Direction OnChangeAction::direction() { return dir; }

uint OnChangeAction::order(const Direction & /*pipelineDir*/) { return SEQUENCE_NR_IN; }

bool OnChangeAction::run(PipelineMessage &pMsg, const Direction & /*pipelineDir*/) {
    if (oldMsg.empty()) {
        // first call, search for header
        auto rosMsg = ros::allocate_message(msgMembers);
        rclcpp::SerializationBase(&msgTypeSupport->type_support_handle)
            .deserialize_message(pMsg.serializedMessage.get(), rosMsg.get());
        auto fishMsg = ros2_babel_fish::CompoundMessage(*msgMemberIntro, rosMsg);

        // find header
        for (size_t i = 0; i < fishMsg.keys().size(); ++i) {
            if ("header" == fishMsg.keyAt(i)) {
                if (0 != i) {
                    std::cout << "OnChange: ⚠ Header found as element number " << i
                              << ". Currently only supported as first element, passing all "
                                 "messages through."
                              << std::endl;
                    active = false;
                }
                headerOffset = DEFAULT_HEADER_OFFSET;
                break;
            }
        }
        auto *cdrMsg = &pMsg.serializedMessage->get_rcl_serialized_message();
        oldMsg.assign(cdrMsg->buffer + headerOffset, cdrMsg->buffer + cdrMsg->buffer_length);
        return true;
    }

    if (!active) {
        return true;
    }

    auto *cdrMsg = &pMsg.serializedMessage->get_rcl_serialized_message();

    // msg has different length
    if (oldMsg.size() + headerOffset != cdrMsg->buffer_length) {
        oldMsg.assign(cdrMsg->buffer + headerOffset, cdrMsg->buffer + cdrMsg->buffer_length);
        return true;
    }

    // check for equality
    if (std::equal(oldMsg.begin(), oldMsg.end(), cdrMsg->buffer + headerOffset)) {
        return false;
    }

    // not equal -> copy new msg
    oldMsg.assign(cdrMsg->buffer + headerOffset, cdrMsg->buffer + cdrMsg->buffer_length);
    return true;
}

}  // namespace dtnproxy::pipeline

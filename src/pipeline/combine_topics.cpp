#include "pipeline/combine_topics.hpp"

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace dtnproxy::pipeline {

void CombineTopicsAction::appendMessage(std::vector<uint8_t>& buffer,
                                        std::shared_ptr<rclcpp::SerializedMessage> msg) {
    uint32_t msgLength =
        htonl(static_cast<uint32_t>(msg->get_rcl_serialized_message().buffer_length));

    // append serialized msg length
    auto* msgLengthPtr = reinterpret_cast<uint8_t*>(&msgLength);
    buffer.insert(buffer.end(), msgLengthPtr, msgLengthPtr + sizeof(msgLength));

    // append msg
    auto cdrMsg = msg->get_rcl_serialized_message();
    buffer.insert(buffer.end(), cdrMsg.buffer, cdrMsg.buffer + cdrMsg.buffer_length);
}

bool CombineTopicsAction::combine(PipelineMessage& pMsg) {
    // check if all msgs exist in store
    bool allInStore = true;
    for (const auto& topic : topics) {
        if (topic != currentTopic && msgStore->find(topic) == msgStore->end()) {
            allInStore = false;
        }
    }

    if (allInStore) {
        // all msgs in store -> ready to send
        std::vector<uint8_t> buffer;
        bool expireAllBundles = pMsg.markExpired;
        for (const auto& topic : topics) {
            if (topic == currentTopic) {
                appendMessage(buffer, pMsg.serializedMessage);
            } else {
                auto msgIt = msgStore->find(topic);
                appendMessage(buffer, msgIt->second.serializedMessage);
                expireAllBundles &= msgIt->second.markExpired;
            }
        }
        msgStore->clear();

        pMsg.serializedMessage->reserve(buffer.size());
        auto* cdrMsg = &pMsg.serializedMessage->get_rcl_serialized_message();
        std::memcpy(cdrMsg->buffer, &buffer.front(), buffer.size());
        cdrMsg->buffer_length = buffer.size();

        pMsg.markExpired = expireAllBundles;

        return true;
    }

    // not all msgs in store yet -> add current msg to store
    msgStore->insert_or_assign(currentTopic, std::move(pMsg));
    return false;
}

bool CombineTopicsAction::split(PipelineMessage& pMsg) {
    auto cdrMsg = pMsg.serializedMessage->get_rcl_serialized_message();

    uint32_t msgLength;
    const auto SIZE_OF_LEN = sizeof(msgLength);
    size_t offset = 0;
    std::vector<uint8_t> buffer;

    for (const auto& topic : topics) {
        std::memcpy(&msgLength, cdrMsg.buffer + offset, SIZE_OF_LEN);
        offset += SIZE_OF_LEN;
        msgLength = ntohl(msgLength);

        if (0 != msgLength) {
            buffer.assign(cdrMsg.buffer + offset, cdrMsg.buffer + offset + msgLength);
            offset += msgLength;

            auto newSerializedMessage = std::make_shared<rclcpp::SerializedMessage>(buffer.size());
            auto* newMsg = &newSerializedMessage->get_rcl_serialized_message();
            std::memcpy(newMsg->buffer, &buffer.front(), buffer.size());
            newMsg->buffer_length = buffer.size();

            if (currentTopic != topic) {
                injectMsgCb(topic, std::move(newSerializedMessage));
            } else {
                pMsg.serializedMessage.swap(newSerializedMessage);
            }
        }
    }

    return true;
}

CombineTopicsAction::CombineTopicsAction(std::string currentTopic, std::vector<std::string> topics,
                                         msgStorePtr_t msgStore, injectMsgCb_t injectMsgCb)
    : topics(std::move(topics)),
      currentTopic(std::move(currentTopic)),
      msgStore(std::move(msgStore)),
      injectMsgCb(std::move(injectMsgCb)) {}

Direction CombineTopicsAction::direction() { return dir; }

uint CombineTopicsAction::order(const Direction& pipelineDir) {
    switch (pipelineDir) {
        case Direction::IN:
            return SEQUENCE_NR_IN;
            break;
        case Direction::OUT:
            return SEQUENCE_NR_OUT;
            break;
        case Direction::INOUT:
        default:
            throw new std::runtime_error("Pipeline should not have Direction INOUT");
    }
}

bool CombineTopicsAction::run(PipelineMessage& pMsg, const Direction& pipelineDir) {
    switch (pipelineDir) {
        case Direction::IN:
            return combine(pMsg);
            break;
        case Direction::OUT:
            return split(pMsg);
            break;
        case Direction::INOUT:
        default:
            throw new std::runtime_error("Pipeline should not have Direction INOUT");
    }
}

}  // namespace dtnproxy::pipeline

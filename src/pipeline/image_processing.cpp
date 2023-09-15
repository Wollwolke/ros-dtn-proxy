#include "pipeline/image_processing.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <qoixx.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

namespace dtnproxy::pipeline {

ImageProcessingAction::ImageProcessingAction(const std::string& msgType) {
    active = (supportedMsgType == msgType);
}

Direction ImageProcessingAction::direction() { return dir; }

uint ImageProcessingAction::order(const Direction& pipelineDir) {
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

bool ImageProcessingAction::run(PipelineMessage& pMsg, const Direction& pipelineDir) {
    if (!active) {
        return true;
    }

    switch (pipelineDir) {
        case Direction::IN:
            return compress(pMsg);
            break;
        case Direction::OUT:
            return decompress(pMsg);
            break;
        case Direction::INOUT:
        default:
            throw new std::runtime_error("Pipeline should not have Direction INOUT");
    }
}

std::tuple<bool, uint8_t> ImageProcessingAction::isEncodingSupported(const std::string& encoding) {
    using namespace sensor_msgs::image_encodings;
    if (isColor(encoding) && bitDepth(encoding) == BIT_DEPTH) {
        if (hasAlpha(encoding)) {
            return {true, QOI_ENCODING_ALPHA};
        }
        return {true, QOI_ENCODING_NO_ALPHA};
    }
    return {false, 0};
}

std::string ImageProcessingAction::getEncodingFromId(uint8_t encodingId) {
    auto iterator = std::find_if(encodingMap.begin(), encodingMap.end(),
                                 [&encodingId](const std::pair<std::string, uint8_t>& entry) {
                                     return entry.second == encodingId;
                                 });
    if (iterator != encodingMap.end()) {
        return iterator->first;
    }
    return {};
}

bool ImageProcessingAction::compress(PipelineMessage& pMsg) {
    using MessageT = sensor_msgs::msg::Image;
    static rclcpp::Serialization<MessageT> imageSerializer;
    static rclcpp::Serialization<std_msgs::msg::Header> headerSerializer;

    MessageT imageMsg;
    imageSerializer.deserialize_message(pMsg.serializedMessage.get(), &imageMsg);

    auto [supported, qoiEncoding] = isEncodingSupported(imageMsg.encoding);
    if (!supported) {
        std::cout << "ImageCompression: Image cannot be compressed, unsuported encoding "
                  << imageMsg.encoding << std::endl;
        return false;
    }

    auto desc = qoixx::qoi::desc{.width = static_cast<std::uint32_t>(imageMsg.width),
                                 .height = static_cast<std::uint32_t>(imageMsg.height),
                                 .channels = qoiEncoding,
                                 .colorspace = qoixx::qoi::colorspace::srgb};
    auto encoded =
        qoixx::qoi::encode<std::vector<uint8_t>>(imageMsg.data.data(), imageMsg.data.size(), desc);

    // add encoding to compressed image
    encoded.insert(encoded.end(), encodingMap.at(imageMsg.encoding));

    // add Message Header to compressed image
    auto serializedHeader = std::make_shared<rclcpp::SerializedMessage>();
    headerSerializer.serialize_message(&imageMsg.header, serializedHeader.get());
    auto cdrHeader = serializedHeader->get_rcl_serialized_message();
    encoded.insert(encoded.end(), cdrHeader.buffer, cdrHeader.buffer + cdrHeader.buffer_length);

    pMsg.serializedMessage->reserve(encoded.size());
    auto* newCdrMsg = &pMsg.serializedMessage->get_rcl_serialized_message();
    newCdrMsg->buffer_length = encoded.size();
    std::move(encoded.begin(), encoded.end(), newCdrMsg->buffer);

    return true;
}

bool ImageProcessingAction::decompress(PipelineMessage& pMsg) {
    static rclcpp::Serialization<std_msgs::msg::Header> headerSerializer;

    auto cdrMsg = pMsg.serializedMessage->get_rcl_serialized_message();

    // split encoding, header
    auto* iterator = std::search(cdrMsg.buffer, cdrMsg.buffer + cdrMsg.buffer_length,
                                 std::begin(QOI_END_TAG), std::end(QOI_END_TAG));
    uint8_t encodingId = *(iterator + QOI_END_TAG.size());

    auto cdrHeaderSize = std::distance(iterator, cdrMsg.buffer + cdrMsg.buffer_length) -
                         QOI_END_TAG.size() - ENCODING_OFFSET;
    rclcpp::SerializedMessage serializedHeader(cdrHeaderSize);
    auto* cdrHeader = &serializedHeader.get_rcl_serialized_message();
    std::memcpy(cdrHeader->buffer, iterator + QOI_END_TAG.size() + ENCODING_OFFSET, cdrHeaderSize);
    cdrHeader->buffer_length = cdrHeaderSize;
    std_msgs::msg::Header header;
    headerSerializer.deserialize_message(&serializedHeader, &header);

    // decode image
    auto image = qoixx::qoi::decode<std::vector<uint8_t>>(
        cdrMsg.buffer, std::distance(cdrMsg.buffer, iterator) + QOI_END_TAG.size());

    // build ROS msg
    sensor_msgs::msg::Image imageMsg;
    imageMsg.header = header;
    imageMsg.height = image.second.height;
    imageMsg.width = image.second.width;
    imageMsg.encoding = getEncodingFromId(encodingId);
    imageMsg.is_bigendian = 0;
    imageMsg.step = image.first.size() / image.second.height;
    imageMsg.data = image.first;

    auto serializedMsg = std::make_shared<rclcpp::SerializedMessage>();
    static rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    serializer.serialize_message(&imageMsg, serializedMsg.get());

    pMsg.serializedMessage.swap(serializedMsg);
    return true;
}

}  // namespace dtnproxy::pipeline

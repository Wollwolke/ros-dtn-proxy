#include "pipeline/lzmh.hpp"

#include <iostream>
#include <stdexcept>

extern "C" {
#include "bit_file_buffer.h"
#include "err_codes.h"
#include "file_buffer.h"
#include "lzmh.h"
}

namespace dtnproxy::pipeline {

LzmhAction::LzmhAction(const std::string &msgType) {
    fbIn = AllocateFileBuffer();
    fbOut = AllocateFileBuffer();
    options.error_log_file = stderr;
    active = !(unSupportedMsgType == msgType);
}

LzmhAction::~LzmhAction() {
    FreeFileBuffer(fbIn);
    FreeFileBuffer(fbOut);
}

Direction LzmhAction::direction() { return dir; }

uint LzmhAction::order(const Direction &pipelineDir) {
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

bool LzmhAction::run(PipelineMessage &pMsg, const Direction &pipelineDir) {
    if (!active) {
        return true;
    }

    auto cdrMsg = pMsg.serializedMessage->get_rcl_serialized_message();

    InitFileBufferInMemory(fbIn, FBM_WRITING, cdrMsg.buffer_length);
    InitFileBufferInMemory(fbOut, FBM_WRITING, cdrMsg.buffer_length);

    WriteFileBuffer(fbIn, cdrMsg.buffer, cdrMsg.buffer_length);
    SetFileBufferMode(fbIn, FBM_READING);

    // Create & Init BitFileBuffer
    auto *bbIn = AllocateBitFileBuffer();
    auto *bbOut = AllocateBitFileBuffer();
    InitBitFileBuffer(bbIn, fbIn);
    InitBitFileBuffer(bbOut, fbOut);

    io_int_t ret;

    switch (pipelineDir) {
        case Direction::IN:
            ret = EncodeLZMH(bbIn, bbOut, &options);
            break;
        case Direction::OUT:
            ret = DecodeLZMH(bbIn, bbOut, &options);
            break;
        case Direction::INOUT:
        default:
            throw new std::runtime_error("Pipeline should not have Direction INOUT");
    }

    if (NO_ERROR != ret) {
        std::cout << "LZMH En-/Decoding: 💥 Error during en-/decoding." << std::endl;
        return false;
    }

    // Read modified data
    SetBitFileBufferMode(bbOut, FBM_READING);
    io_int_t bytes;
    uint8_t bits;
    GetActualBitFileSize(bbOut, &bytes, &bits);

    auto bitsToRead = bytes * BITS_IN_BYTE + bits;
    auto bytesToReserve = bytes + (bits == 0 ? 0 : 1);

    pMsg.serializedMessage->reserve(bytesToReserve);
    auto *newCdrMsg = &pMsg.serializedMessage->get_rcl_serialized_message();
    newCdrMsg->buffer_length = bytesToReserve;

    ret = ReadBitFileBuffer(bbOut, newCdrMsg->buffer, bitsToRead);

    if (ret != bitsToRead) {
        std::cout << "LZMH En-/Decoding: 💥 Error reading modified data." << std::endl;
        return false;
    }

    // Destroy BitFileBuffer
    UninitBitFileBuffer(bbIn);
    UninitBitFileBuffer(bbOut);
    FreeBitFileBuffer(bbIn);
    FreeBitFileBuffer(bbOut);

    // Reset FileBuffer
    UninitFileBuffer(fbIn);
    UninitFileBuffer(fbOut);

    return true;
}

}  // namespace dtnproxy::pipeline

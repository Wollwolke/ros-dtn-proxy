#include "stats_recorder.hpp"

#include <array>
#include <cstddef>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <sstream>

#include "ros/dtn_msg_type.hpp"

namespace dtnproxy {

std::string StatsRecorder::timestamp() {
    using Clock = std::chrono::system_clock;

    auto now = Clock::now();
    auto epoch_seconds = std::chrono::system_clock::to_time_t(now);
    // Convert back to a time_point to get the time truncated to whole seconds
    auto truncated = std::chrono::system_clock::from_time_t(epoch_seconds);
    auto delta_us = std::chrono::duration_cast<std::chrono::microseconds>(now - truncated).count();

    std::stringstream timeStream;
    timeStream << std::put_time(std::localtime(&epoch_seconds), "%FT%T");
    timeStream << "." << std::fixed << std::setw(6) << std::setfill('0') << delta_us;
    timeStream << std::put_time(std::localtime(&epoch_seconds), "%z") << ";";

    return timeStream.str();
}

StatsRecorder::StatsRecorder(const std::string& statsDir) {
    std::filesystem::path statsPath(statsDir);
    std::filesystem::create_directories(statsPath);
    file.exceptions(std::ios::failbit);

    auto time = std::time(nullptr);

    const auto lenOfDateString = (4 + 1 + 2 + 1 + 2) + 1 + (2 + 1 + 2 + 1 + 2) + 1;
    std::array<char, lenOfDateString> fileName;
    std::strftime(fileName.data(), lenOfDateString, "%Y-%m-%d_%H-%M-%S", std::localtime(&time));

    auto filePath = statsPath;

    filePath.append(fileName.data()).replace_extension(".log");
    try {
        file.open(filePath);
    } catch (const std::ios_base::failure& e) {
        std::cerr << "StatsRecorder: 💥 Error opening logfile " << filePath << ": " << e.what()
                  << std::endl;
        throw;
    }
}

StatsRecorder::~StatsRecorder() { file.close(); }

// TODO: refactor this...

void StatsRecorder::rosReceived(const std::string& topic, const std::string& msgType, uint32_t size,
                                ros::DtnMsgType rosType, size_t msgHash) {
    std::stringstream logMsg;
    logMsg << timestamp() << "ROS;";

    switch (rosType) {
        case ros::DtnMsgType::TOPIC: {
            logMsg << "TOPIC;"
                   << "SUB;";
        } break;
        case ros::DtnMsgType::REQUEST: {
            logMsg << "SERVER;"
                   << "REQUEST;";
        } break;
        case ros::DtnMsgType::RESPONSE: {
            logMsg << "CLIENT;"
                   << "RESPONSE;";
        } break;
        case ros::DtnMsgType::INVALID:
        default:
            return;
    }
    logMsg << topic << ";" << msgType << ";" << size << ";" << msgHash;
    file << logMsg.rdbuf() << std::endl;
}

void StatsRecorder::rosSent(const std::string& topic, const std::string& msgType, uint32_t size,
                            ros::DtnMsgType rosType, size_t msgHash) {
    std::stringstream logMsg;
    logMsg << timestamp() << "ROS;";

    switch (rosType) {
        case ros::DtnMsgType::TOPIC: {
            logMsg << "TOPIC;"
                   << "PUB;";
        } break;
        case ros::DtnMsgType::REQUEST: {
            logMsg << "CLIENT;"
                   << "REQUEST;";
        } break;
        case ros::DtnMsgType::RESPONSE: {
            logMsg << "SERVER;"
                   << "RESPONSE;";
        } break;
        case ros::DtnMsgType::INVALID:
        default:
            return;
    }
    logMsg << topic << ";" << msgType << ";" << size << ";" << msgHash;
    file << logMsg.rdbuf() << std::endl;
}

void StatsRecorder::dtnReceived(const std::string& endpoint, const std::string& msgType,
                                uint32_t size, ros::DtnMsgType rosType) {
    std::stringstream logMsg;
    logMsg << timestamp() << "DTN;";

    switch (rosType) {
        case ros::DtnMsgType::TOPIC: {
            logMsg << "TOPIC;"
                   << "PUB;";
        } break;
        case ros::DtnMsgType::REQUEST: {
            logMsg << "CLIENT;"
                   << "REQUEST;";
        } break;
        case ros::DtnMsgType::RESPONSE: {
            logMsg << "SERVER;"
                   << "RESPONSE;";
        } break;
        case ros::DtnMsgType::INVALID:
        default:
            return;
    }
    logMsg << endpoint << ";" << msgType << ";" << size;
    file << logMsg.rdbuf() << std::endl;
}

void StatsRecorder::dtnSent(const std::string& endpoint, const std::string& msgType, uint32_t size,
                            ros::DtnMsgType rosType) {
    std::stringstream logMsg;
    logMsg << timestamp() << "DTN;";

    switch (rosType) {
        case ros::DtnMsgType::TOPIC: {
            logMsg << "TOPIC;"
                   << "SUB;";
        } break;
        case ros::DtnMsgType::REQUEST: {
            logMsg << "SERVER;"
                   << "REQUEST;";
        } break;
        case ros::DtnMsgType::RESPONSE: {
            logMsg << "CLIENT;"
                   << "RESPONSE;";
        } break;
        case ros::DtnMsgType::INVALID:
        default:
            return;
    }
    logMsg << endpoint << ";" << msgType << ";" << size;
    file << logMsg.rdbuf() << std::endl;
}

}  // namespace dtnproxy

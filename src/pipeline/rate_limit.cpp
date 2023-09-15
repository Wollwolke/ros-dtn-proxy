#include "pipeline/rate_limit.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

RateLimitAction::RateLimitAction(std::vector<std::string> params) {
    if (1 != params.size()) {
        std::cout << "RateLimit: 💥 Exactly one argument required." << std::endl;
    } else {
        try {
            deltaT = std::stoi(params[0]) * MS_IN_SECOND;
        } catch (const std::logic_error& e) {
            std::cout << "RateLimit: 💥 Argument is not a number: " << e.what() << std::endl;
        }
    }
}

Direction RateLimitAction::direction() { return dir; }

uint RateLimitAction::order(const Direction& /*pipelineDir*/) { return SEQUENCE_NR_IN; }

bool RateLimitAction::run(PipelineMessage& /*pMsg*/, const Direction& /*pipelineDir*/) {
    using namespace std::chrono;

    auto timeNow = steady_clock::now();
    if (duration_cast<milliseconds>(timeNow - lastMsgSentTime).count() > deltaT) {
        lastMsgSentTime = timeNow;
        return true;
    }
    return false;
}

}  // namespace dtnproxy::pipeline

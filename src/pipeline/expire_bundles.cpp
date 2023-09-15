#include "pipeline/expire_bundles.hpp"

namespace dtnproxy::pipeline {

Direction ExpireBundlesAction::direction() { return dir; }

uint ExpireBundlesAction::order(const Direction& /*pipelineDir*/) { return SEQUENCE_NR_IN; }

bool ExpireBundlesAction::run(PipelineMessage& pMsg, const Direction& /*pipelineDir*/) {
    pMsg.markExpired = true;
    return true;
}

}  // namespace dtnproxy::pipeline

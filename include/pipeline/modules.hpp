#pragma once

#include <map>
#include <string>
namespace dtnproxy::pipeline {

enum Module {
    RATE_LIMIT,
    COMPRESS,
    IMG_COMPRESS,
    EXPIRE,
    ON_CHANGE,
    COMBINE,
    COMBINE_TF,
};

const std::map<std::string, Module> moduleMapping{
    {"RateLimit", Module::RATE_LIMIT},       {"Compress", Module::COMPRESS},
    {"ImageCompress", Module::IMG_COMPRESS}, {"Expire", Module::EXPIRE},
    {"OnChange", Module::ON_CHANGE},         {"Combine", Module::COMBINE},
    {"CombineTF", Module::COMBINE_TF}};

}  // namespace dtnproxy::pipeline

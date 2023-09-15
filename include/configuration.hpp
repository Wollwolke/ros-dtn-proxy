#pragma once

#include <exception>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "logger.hpp"
#include "pipeline/modules.hpp"

#define TOML11_NO_ERROR_PREFIX
#include <toml.hpp>

namespace dtnproxy::conf {

using Module = struct Module {
    pipeline::Module enumId;
    std::vector<std::string> params;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Module, enumId, params)

using RosConfig = struct RosConfig {
    using RosTopic = struct RosTopic {
        std::string name;
        std::string type;
        std::string profile;
        void from_toml(const toml::value& v);
    };
    using RosService = RosTopic;

    std::vector<RosTopic> subTopics;
    std::vector<RosService> clients;

    std::map<std::string, std::vector<Module>> profiles;
};

using DtnConfig = struct DtnConfig {
    std::string address;
    uint16_t port;
    std::string remoteNodeId;
    uint32_t lifetime;
};

using Config = struct Config {
    std::string statsDir;
    DtnConfig dtn;
    RosConfig ros;
};

using RemoteConfig = struct RemoteConfig {
    using RosInterface = struct RosInterface {
        bool isService;
        std::string topic;
        std::string type;
        std::vector<Module> modules;
        RosInterface(bool isService, std::string topic, std::string type,
                     std::vector<Module> modules)
            : isService(isService),
              topic(std::move(topic)),
              type(std::move(type)),
              modules(std::move(modules)){};
        RosInterface() = default;
    };

    std::vector<RosInterface> interfaces;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RemoteConfig::RosInterface, isService, topic, type, modules)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RemoteConfig, interfaces)

class ConfigurationReader {
private:
    static DtnConfig initDtnConfig(const toml::value& config, Logger& log);
    static RosConfig initRosConfig(const toml::value& config, Logger& log);
    static void initProfilesConfig(const toml::value& config, RosConfig& rosConfig);
    static pipeline::Module resolveStringModule(const std::string& moduleName);
    static std::vector<std::string> collectRequiredProfiles(const RosConfig& rosConfig);

public:
    static Config readConfigFile(const std::string& filePath);
    static RemoteConfig getRemoteConfig(const RosConfig& rosConfig);
};

class ConfigException : public std::exception {
private:
    std::string message;

public:
    ConfigException(std::string msg = "Error while parsing configuration!") : message(msg) {}
    const char* what() { return message.c_str(); }
};

}  // namespace dtnproxy::conf

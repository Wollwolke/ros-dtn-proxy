#include "configuration.hpp"

#include <iostream>
#include <vector>

#include "pipeline/modules.hpp"

namespace dtnproxy::conf {

Config ConfigurationReader::readConfigFile(const std::string& filePath) {
    auto log = Logger("ConfigReader");

    toml::value config;
    try {
        config = toml::parse(filePath);
        log.DBG() << config;
    } catch (const std::exception& err) {
        log.FATAL() << err.what();
        throw ConfigException();
    }

    const DtnConfig dtn = initDtnConfig(config, log);
    const RosConfig ros = initRosConfig(config, log);
    const Config cfg{
        toml::find_or<std::string>(config, "statsDir",
                                   "/var/log/dtnproxy/stats/"),  // std::string statsPath;
        dtn,                                                     // DtnConfig dtn,
        ros,                                                     // RosConfig ros,
    };

    return cfg;
}

RemoteConfig ConfigurationReader::getRemoteConfig(const RosConfig& rosConfig) {
    RemoteConfig remoteConfig;
    for (const auto& [topic, type, profile] : rosConfig.subTopics) {
        std::vector<Module> modules;
        if (!profile.empty()) {
            modules = rosConfig.profiles.at(profile);
        }
        remoteConfig.interfaces.emplace_back(false, topic, type, modules);
    }

    for (const auto& [topic, type, profile] : rosConfig.clients) {
        std::vector<Module> modules;
        if (!profile.empty()) {
            modules = rosConfig.profiles.at(profile);
        }
        remoteConfig.interfaces.emplace_back(true, topic, type, modules);
    }
    return remoteConfig;
}

DtnConfig ConfigurationReader::initDtnConfig(const toml::value& config, Logger& log) {
    DtnConfig dtnConfig;
    if (config.contains("dtn")) {
        auto dtn = toml::find(config, "dtn");
        try {
            if (dtn.contains("remoteNodeId")) {
                dtnConfig.remoteNodeId = toml::find<std::string>(dtn, "remoteNodeId");
            } else {
                log.FATAL() << "Missing mandatory parameter: dtn.remoteNodeId !";
                throw ConfigException();
            }
            dtnConfig.lifetime = toml::find_or<uint32_t>(dtn, "lifetime", 5 * 60);
            dtnConfig.address = toml::find_or<std::string>(dtn, "dtndAddress", "127.0.0.1");
            dtnConfig.port = toml::find_or<uint16_t>(dtn, "dtndPort", 3000);
        } catch (const toml::exception& err) {
            log.ERR() << err.what();
            throw ConfigException();
        }
    }
    return dtnConfig;
}

RosConfig ConfigurationReader::initRosConfig(const toml::value& config, Logger& log) {
    RosConfig rosConfig;
    bool foundTopics = false;
    if (config.contains("ros")) {
        const auto ros = toml::find(config, "ros");
        try {
            if (ros.contains("topics")) {
                rosConfig.subTopics = toml::find<std::vector<RosConfig::RosTopic>>(ros, "topics");
                foundTopics = true;
            }
            if (ros.contains("services")) {
                rosConfig.clients = toml::find<std::vector<RosConfig::RosService>>(ros, "services");
                foundTopics = true;
            }
        } catch (const toml::exception& err) {
            log.ERR() << err.what();
            throw ConfigException();
        }
    }
    if (foundTopics) {
        // load profiles
        if (config.contains("profile")) {
            try {
                initProfilesConfig(config, rosConfig);
            } catch (const std::exception& err) {
                log.ERR() << err.what();
                throw ConfigException();
            }
        }
        auto requiredProfiles = collectRequiredProfiles(rosConfig);
        for (const auto& profile : requiredProfiles) {
            if (rosConfig.profiles.find(profile) == rosConfig.profiles.end()) {
                throw ConfigException(std::string("Required profile not found: ").append(profile));
            }
        }
    } else {
        log.WARN() << "No topics/services to forward found in config!";
    }
    return rosConfig;
}

void ConfigurationReader::initProfilesConfig(const toml::value& config, RosConfig& rosConfig) {
    const auto profiles = toml::find<std::vector<toml::value>>(config, "profile");
    for (const auto& profile : profiles) {
        auto profileName = toml::find<std::string>(profile, "name");
        std::vector<Module> modules;
        const auto tomlModules = toml::find<std::vector<toml::value>>(profile, "module");
        for (const auto& module : tomlModules) {
            Module mod;
            auto moduleName = toml::find<std::string>(module, "name");
            mod.enumId = resolveStringModule(moduleName);
            mod.params = toml::find<std::vector<std::string>>(module, "params");
            modules.push_back(mod);
        }
        rosConfig.profiles.insert_or_assign(profileName, modules);
    }
}

pipeline::Module ConfigurationReader::resolveStringModule(const std::string& moduleName) {
    using pipeline::Module;
    using pipeline::moduleMapping;

    auto itr = moduleMapping.find(moduleName);
    if (itr != moduleMapping.end()) {
        return itr->second;
    }
    throw ConfigException(std::string("Module not found: ").append(moduleName));
}

std::vector<std::string> ConfigurationReader::collectRequiredProfiles(const RosConfig& rosConfig) {
    std::vector<std::string> profiles;
    std::vector<std::vector<RosConfig::RosTopic>> temp;
    temp.emplace_back(rosConfig.subTopics);
    temp.emplace_back(rosConfig.clients);

    for (const auto& vec : temp) {
        for (const auto& topic : vec) {
            if (!topic.profile.empty()) {
                profiles.push_back(topic.profile);
            }
        }
    }

    return profiles;
}

void RosConfig::RosTopic::from_toml(const toml::value& v) {
    try {
        auto tmp = toml::get<std::vector<std::string>>(v);
        if (tmp.size() < 2) {
            throw ConfigException("Malformed Config in [ros]");
        }
        this->name = tmp[0];
        this->type = tmp[1];
        this->profile = (tmp.size() == 3) ? tmp[2] : "";
    } catch (std::exception& e) {
        throw ConfigException(e.what());
    }
}

}  // namespace dtnproxy::conf

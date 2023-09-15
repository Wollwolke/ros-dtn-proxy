#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "common.hpp"
#include "configuration.hpp"
#include "dtnd_client.hpp"
#include "ros/transfer.hpp"
#include "stats_recorder.hpp"

namespace dtnproxy {

class DtnProxy : public rclcpp::Node {
private:
    using DtnEndpoint = std::pair<std::string, ros::DtnMsgType>;

    conf::Config config;

    std::shared_ptr<DtndClient> dtn;
    std::unique_ptr<ros::Transfer> ros;

    void fatalShutdown(const std::string& reason) {
        RCLCPP_FATAL_STREAM(this->get_logger(), reason);
        dtn.reset();
        exit(EXIT_FAILURE);
    }

    std::vector<DtnEndpoint> buildDtnEndpoints() {
        using ros::DtnMsgType;
        std::vector<DtnEndpoint> result;

        std::transform(
            config.ros.clients.begin(), config.ros.clients.end(), std::back_inserter(result),
            [&](auto const& topic) { return std::pair(topic.name, DtnMsgType::REQUEST); });
        return result;
    }

    void loadConfig() {
        auto packageShareDirectory =
            ament_index_cpp::get_package_share_directory(common::PACKAGE_NAME);
        auto parameterDesc = rcl_interfaces::msg::ParameterDescriptor{};
        parameterDesc.description = "Absolute path to the configuration file.";

        this->declare_parameter("configurationPath", packageShareDirectory + "/config/node0.toml",
                                parameterDesc);
        auto path =
            this->get_parameter("configurationPath").get_parameter_value().get<std::string>();

        try {
            config = conf::ConfigurationReader::readConfigFile(path);
        } catch (conf::ConfigException& e) {
            fatalShutdown(e.what());
        }
    }

public:
    DtnProxy() : Node(common::DEFAULT_NODE_NAME) {
        loadConfig();

        dtn = std::make_shared<DtndClient>(config.dtn);
        ros = std::make_unique<ros::Transfer>(*this, config.ros, dtn);

        dtn->setMessageHandler(
            std::bind(&ros::Transfer::onDtnMessage, ros.get(), std::placeholders::_1));
        ros->enableStatsRecorder(std::make_shared<StatsRecorder>(config.statsDir));

        auto dtnEndpoints = buildDtnEndpoints();
        dtn->registerEndpoints(dtnEndpoints);

        auto remoteConfig = conf::ConfigurationReader::getRemoteConfig(config.ros);
        if (!remoteConfig.interfaces.empty()) {
            dtn->sendRemoteConfig(nlohmann::json::to_cbor(remoteConfig));
        }

        ros->initSubscribersAndClients();

        RCLCPP_INFO_STREAM(this->get_logger(), "DtnProxy up.");
    }
};

}  // namespace dtnproxy

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dtnproxy::DtnProxy>());
    rclcpp::shutdown();
    return 0;
}

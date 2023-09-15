#include "logger.hpp"

namespace dtnproxy {

Logger::Logger(const std::string& moduleName, const std::string& loggerName)
    : rosLogger(rclcpp::get_logger(loggerName)), name(moduleName) {}

Logger::Base::Base(const rclcpp::Logger& rosLogger, const std::string& name)
    : rosLogger(rosLogger) {
    namePrefix = "[" + name + "]: ";
}

Logger::Debug::Debug(const rclcpp::Logger& rosLogger, const std::string& name)
    : Logger::Base(rosLogger, name) {}
Logger::Debug::~Debug() {
    std::string str = namePrefix + ss.str();
    if (!str.empty()) RCLCPP_DEBUG_STREAM(rosLogger, str);
}

Logger::Info::Info(const rclcpp::Logger& rosLogger, const std::string& name)
    : Logger::Base(rosLogger, name) {}
Logger::Info::~Info() {
    std::string str = namePrefix + ss.str();
    if (!str.empty()) RCLCPP_INFO_STREAM(rosLogger, str);
}

Logger::Warning::Warning(const rclcpp::Logger& rosLogger, const std::string& name)
    : Logger::Base(rosLogger, name) {}
Logger::Warning::~Warning() {
    std::string str = namePrefix + ss.str();
    if (!str.empty()) RCLCPP_WARN_STREAM(rosLogger, str);
}

Logger::Error::Error(const rclcpp::Logger& rosLogger, const std::string& name)
    : Logger::Base(rosLogger, name) {}
Logger::Error::~Error() {
    std::string str = namePrefix + ss.str();
    if (!str.empty()) RCLCPP_ERROR_STREAM(rosLogger, str);
}

Logger::Fatal::Fatal(const rclcpp::Logger& rosLogger, const std::string& name)
    : Logger::Base(rosLogger, name) {}
Logger::Fatal::~Fatal() {
    std::string str = namePrefix + ss.str();
    if (!str.empty()) RCLCPP_FATAL_STREAM(rosLogger, str);
}

}  // namespace dtnproxy

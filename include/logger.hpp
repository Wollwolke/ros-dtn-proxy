#pragma once
#include <functional>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "common.hpp"
namespace dtnproxy {

class Logger {
private:
    rclcpp::Logger rosLogger;
    std::string name;

    class Base {
    protected:
        rclcpp::Logger rosLogger;
        std::string namePrefix;
        std::stringstream ss;

    public:
        Base(const rclcpp::Logger& rosLogger, const std::string& name);

        template <typename T>
        Base& operator<<(const T& value) {
            ss << value;
            return *this;
        }
    };

    class Debug : public Base {
    public:
        Debug(const rclcpp::Logger& rosLogger, const std::string& name);
        ~Debug();
    };

    class Info : public Base {
    public:
        Info(const rclcpp::Logger& rosLogger, const std::string& name);
        ~Info();
    };

    class Warning : public Base {
    public:
        Warning(const rclcpp::Logger& rosLogger, const std::string& name);
        ~Warning();
    };

    class Error : public Base {
    public:
        Error(const rclcpp::Logger& rosLogger, const std::string& name);
        ~Error();
    };

    class Fatal : public Base {
    public:
        Fatal(const rclcpp::Logger& rosLogger, const std::string& name);
        ~Fatal();
    };

public:
    explicit Logger(const std::string& moduleName,
                    const std::string& loggerName = common::DEFAULT_NODE_NAME);
    ~Logger() = default;

    Debug DBG() { return Debug(rosLogger, name); }
    Info INFO() { return Info(rosLogger, name); }
    Warning WARN() { return Warning(rosLogger, name); }
    Error ERR() { return Error(rosLogger, name); }
    Fatal FATAL() { return Fatal(rosLogger, name); }
};

}  // namespace dtnproxy

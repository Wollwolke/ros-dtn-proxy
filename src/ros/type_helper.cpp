#include "ros/type_helper.hpp"

#include <cstring>
#include <iostream>
#include <rclcpp/typesupport_helpers.hpp>
#include <string>

namespace dtnproxy::ros {

using rosidl_typesupport_introspection_cpp::MessageMembers;

// ----------
// Copy of
// https://github.com/ros2/rclcpp/blob/33dae5d67/rclcpp/src/rclcpp/typesupport_helpers.cpp#L69-L92
static std::tuple<std::string, std::string, std::string> extract_type_identifier(
    const std::string& full_type) {
    char type_separator = '/';
    auto sep_position_back = full_type.find_last_of(type_separator);
    auto sep_position_front = full_type.find_first_of(type_separator);
    if (sep_position_back == std::string::npos || sep_position_back == 0 ||
        sep_position_back == full_type.length() - 1) {
        throw std::runtime_error(
            "Message type is not of the form package/type and cannot be processed");
    }

    std::string package_name = full_type.substr(0, sep_position_front);
    std::string middle_module = "";
    if (sep_position_back - sep_position_front > 0) {
        middle_module =
            full_type.substr(sep_position_front + 1, sep_position_back - sep_position_front - 1);
    }
    std::string type_name = full_type.substr(sep_position_back + 1);

    return std::make_tuple(package_name, middle_module, type_name);
}
// End of Copy
// ----------

// ----------
// Copy of
// https://github.com/foxglove/ros-foxglove-bridge/blob/f4b2f68cd/ros2_foxglove_bridge/src/generic_client.cpp#L45-L92
std::shared_ptr<void> allocate_message(const MessageMembers* members) {
    void* buffer = malloc(members->size_of_);
    if (buffer == nullptr) {
        throw std::runtime_error("Failed to allocate memory");
    }
    std::memset(buffer, 0, members->size_of_);
    members->init_function(buffer, rosidl_runtime_cpp::MessageInitialization::ALL);
    return std::shared_ptr<void>(buffer, free);
}

std::string getTypeIntrospectionSymbolName(const std::string& serviceType) {
    const auto [pkgName, middleModule, typeName] = extract_type_identifier(serviceType);

    return std::string(TYPESUPPORT_INTROSPECTION_LIB_NAME) + "__get_service_type_support_handle__" +
           pkgName + "__" + (middleModule.empty() ? "srv" : middleModule) + "__" + typeName;
}

std::string getServiceTypeSupportHandleSymbolName(const std::string& serviceType) {
    const auto [pkgName, middleModule, typeName] = extract_type_identifier(serviceType);
    const auto lengthPrefixedString = [](const std::string& s) {
        return std::to_string(s.size()) + s;
    };

    return "_ZN" + lengthPrefixedString(TYPESUPPORT_LIB_NAME) +
           lengthPrefixedString("get_service_type_support_handle") + "IN" +
           lengthPrefixedString(pkgName) +
           lengthPrefixedString(middleModule.empty() ? "srv" : middleModule) +
           lengthPrefixedString(typeName) + "EEEPK" +
           lengthPrefixedString("rosidl_service_type_support_t") + "v";
}
// End of Copy
// ----------

// Based on
// https://github.com/foxglove/ros-foxglove-bridge/blob/f4b2f68cd/ros2_foxglove_bridge/src/generic_client.cpp

void ServiceTypeSupport::initTypeHandles(const std::string& serviceType) {
    const auto requestTypeName = serviceType + "_Request";
    const auto responseTypeName = serviceType + "_Response";

    _typeSupportLib = rclcpp::get_typesupport_library(serviceType, TYPESUPPORT_LIB_NAME);
    _typeIntrospectionLib =
        rclcpp::get_typesupport_library(serviceType, TYPESUPPORT_INTROSPECTION_LIB_NAME);
    if (!_typeSupportLib || !_typeIntrospectionLib) {
        throw std::runtime_error("Failed to load shared library for service type " + serviceType);
    }

    const auto typesupportSymbolName = getServiceTypeSupportHandleSymbolName(serviceType);
    if (!_typeSupportLib->has_symbol(typesupportSymbolName)) {
        throw std::runtime_error("Failed to find symbol '" + typesupportSymbolName + "' in " +
                                 _typeSupportLib->get_library_path());
    }

    const rosidl_service_type_support_t* (*get_ts)() = nullptr;
    _serviceTypeSupportHdl =
        (reinterpret_cast<decltype(get_ts)>(_typeSupportLib->get_symbol(typesupportSymbolName)))();

    const auto typeinstrospection_symbol_name = getTypeIntrospectionSymbolName(serviceType);

    // This will throw runtime_error if the symbol was not found.
    _typeIntrospectionHdl = (reinterpret_cast<decltype(get_ts)>(
        _typeIntrospectionLib->get_symbol(typeinstrospection_symbol_name)))();
    _requestTypeSupportHdl =
        rclcpp::get_typesupport_handle(requestTypeName, TYPESUPPORT_LIB_NAME, *_typeSupportLib);
    _responseTypeSupportHdl =
        rclcpp::get_typesupport_handle(responseTypeName, TYPESUPPORT_LIB_NAME, *_typeSupportLib);
}

ServiceTypeSupport::ServiceTypeSupport(const std::string& serviceType) {
    initTypeHandles(serviceType);
}

}  // namespace dtnproxy::ros

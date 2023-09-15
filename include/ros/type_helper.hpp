#pragma once
#include <memory>
#include <rcpputils/shared_library.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>
#include <string>

namespace dtnproxy::ros {

constexpr auto TYPESUPPORT_INTROSPECTION_LIB_NAME = "rosidl_typesupport_introspection_cpp";
constexpr auto TYPESUPPORT_LIB_NAME = "rosidl_typesupport_cpp";

// ----------
// Copy of
// https://github.com/foxglove/ros-foxglove-bridge/blob/f4b2f68cd/ros2_foxglove_bridge/src/generic_client.cpp#L45-L92

std::shared_ptr<void> allocate_message(
    const rosidl_typesupport_introspection_cpp::MessageMembers* members);

std::string getTypeIntrospectionSymbolName(const std::string& serviceType);

/**
 * The default symbol names for getting type support handles for services are missing from the
 * rosidl_typesupport_cpp shared libraries, see
 * https://github.com/ros2/rosidl_typesupport/issues/122
 *
 * We can however, as a (hacky) workaround, use other symbols defined in the shared library.
 * With `nm -C -D /opt/ros/humble/lib/libtest_msgs__rosidl_typesupport_cpp.so` we see that there is
 * `rosidl_service_type_support_t const*
 * rosidl_typesupport_cpp::get_service_type_support_handle<test_msgs::srv::BasicTypes>()` which
 * mangled becomes
 * `_ZN22rosidl_typesupport_cpp31get_service_type_support_handleIN9test_msgs3srv10BasicTypesEEEPK29rosidl_service_type_support_tv`
 * This is the same for galactic, humble and rolling (tested with gcc / clang)
 *
 * This function produces the mangled symbol name for a given service type.
 *
 * \param[in] serviceType The service type, e.g. "test_msgs/srv/BasicTypes"
 * \return Symbol name for getting the service type support handle
 */
std::string getServiceTypeSupportHandleSymbolName(const std::string& serviceType);

// End of Copy
// ----------

// Based on
// https://github.com/foxglove/ros-foxglove-bridge/blob/f4b2f68cd/ros2_foxglove_bridge/src/generic_client.cpp

class ServiceTypeSupport {
private:
    void initTypeHandles(const std::string &serviceType);

protected:
    std::shared_ptr<rcpputils::SharedLibrary> _typeSupportLib;
    std::shared_ptr<rcpputils::SharedLibrary> _typeIntrospectionLib;
    const rosidl_service_type_support_t *_serviceTypeSupportHdl;
    const rosidl_message_type_support_t *_responseTypeSupportHdl;
    const rosidl_message_type_support_t *_requestTypeSupportHdl;
    const rosidl_service_type_support_t *_typeIntrospectionHdl;

    ServiceTypeSupport(const std::string &serviceType);
};

}  // namespace dtnproxy::ros

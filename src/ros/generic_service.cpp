#include "ros/generic_service.hpp"

#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include "ros/type_helper.hpp"

// Based on
// https://github.com/foxglove/ros-foxglove-bridge/blob/f4b2f68cd/ros2_foxglove_bridge/src/generic_client.cpp

namespace dtnproxy::ros {

using rosidl_typesupport_introspection_cpp::ServiceMembers;

GenericService::GenericService(std::shared_ptr<rcl_node_t> nodeHandle,
                               const std::string& serviceName, const std::string& serviceType,
                               CallbackType callback, rcl_service_options_t& serviceOptions)
    : ServiceBase(nodeHandle), ServiceTypeSupport(serviceType), callback(callback) {
    // rcl does the static memory allocation here
    service_handle_ = std::shared_ptr<rcl_service_t>(
        new rcl_service_t, [handle = node_handle_, serviceName](rcl_service_t* service) {
            if (rcl_service_fini(service, handle.get()) != RCL_RET_OK) {
                RCLCPP_ERROR(rclcpp::get_node_logger(handle.get()).get_child("rclcpp"),
                             "Error in destruction of rcl service handle: %s",
                             rcl_get_error_string().str);
                rcl_reset_error();
            }
            delete service;
        });
    *service_handle_.get() = rcl_get_zero_initialized_service();

    rcl_ret_t ret = rcl_service_init(service_handle_.get(), nodeHandle.get(),
                                     _serviceTypeSupportHdl, serviceName.c_str(), &serviceOptions);
    if (ret != RCL_RET_OK) {
        if (ret == RCL_RET_SERVICE_NAME_INVALID) {
            auto rcl_node_handle = this->get_rcl_node_handle();
            // this will throw on any validation problem
            rcl_reset_error();
            rclcpp::expand_topic_or_service_name(serviceName, rcl_node_get_name(rcl_node_handle),
                                                 rcl_node_get_namespace(rcl_node_handle), true);
        }
        rclcpp::exceptions::throw_from_rcl_error(ret, "Could not create service");
    }
}

std::shared_ptr<rmw_request_id_t> GenericService::create_request_header() {
    return std::make_shared<rmw_request_id_t>();
}

void GenericService::handle_request(std::shared_ptr<rmw_request_id_t> requestHeader,
                                    std::shared_ptr<void> request) {
    auto serRequest = std::make_shared<rclcpp::SerializedMessage>();
    rmw_ret_t result = rmw_serialize(request.get(), _requestTypeSupportHdl,
                                     &serRequest->get_rcl_serialized_message());
    if (result != RMW_RET_OK) {
        RCLCPP_ERROR(rclcpp::get_node_logger(node_handle_.get()).get_child("rclcpp"),
                     "Failed to serialize service request. Ignoring...");
        return;
    }

    callback(requestHeader, serRequest);
}

void GenericService::send_response(rmw_request_id_t& requestId,
                                   std::shared_ptr<rclcpp::SerializedMessage> response) {
    auto srv_members = static_cast<const ServiceMembers*>(_typeIntrospectionHdl->data);
    auto rosMsg = allocate_message(srv_members->response_members_);

    const rmw_serialized_message_t* rclResponse = &response->get_rcl_serialized_message();

    rcl_ret_t result;
    result = rmw_deserialize(rclResponse, _responseTypeSupportHdl, rosMsg.get());
    if (RCL_RET_OK != result) {
        rclcpp::exceptions::throw_from_rcl_error(result, "Failed to desirialize response");
    }
    rcl_ret_t ret = rcl_send_response(get_service_handle().get(), &requestId, rosMsg.get());

    if (ret != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to send response");
    }
}

bool GenericService::take_request(std::shared_ptr<void> requestOut,
                                  rmw_request_id_t& requestIdOut) {
    return this->take_type_erased_request(requestOut.get(), requestIdOut);
}

std::shared_ptr<void> GenericService::create_request() {
    auto srv_members = static_cast<const ServiceMembers*>(_typeIntrospectionHdl->data);
    return allocate_message(srv_members->request_members_);
}

}  // namespace dtnproxy::ros

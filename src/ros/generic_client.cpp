#include "ros/generic_client.hpp"

#include <future>
#include <iostream>
#include <rclcpp/client.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include "ros/type_helper.hpp"

// Based on
// https://github.com/foxglove/ros-foxglove-bridge/blob/f4b2f68cd/ros2_foxglove_bridge/src/generic_client.cpp

namespace dtnproxy::ros {

using rosidl_typesupport_introspection_cpp::ServiceMembers;

GenericClient::GenericClient(rclcpp::node_interfaces::NodeBaseInterface* nodeBase,
                             rclcpp::node_interfaces::NodeGraphInterface::SharedPtr nodeGraph,
                             const std::string& serviceName, const std::string& serviceType,
                             rcl_client_options_t& clientOptions)
    : rclcpp::ClientBase(nodeBase, nodeGraph), ServiceTypeSupport(serviceType) {
    rcl_ret_t ret = rcl_client_init(this->get_client_handle().get(), this->get_rcl_node_handle(),
                                    _serviceTypeSupportHdl, serviceName.c_str(), &clientOptions);
    if (ret != RCL_RET_OK) {
        if (ret == RCL_RET_SERVICE_NAME_INVALID) {
            auto rcl_node_handle = this->get_rcl_node_handle();
            // this will throw on any validation problem
            rcl_reset_error();
            rclcpp::expand_topic_or_service_name(serviceName, rcl_node_get_name(rcl_node_handle),
                                                 rcl_node_get_namespace(rcl_node_handle), true);
        }
        rclcpp::exceptions::throw_from_rcl_error(ret, "Could not create client");
    }
}

std::shared_ptr<void> GenericClient::create_response() {
    auto srv_members = static_cast<const ServiceMembers*>(_typeIntrospectionHdl->data);
    return allocate_message(srv_members->response_members_);
}

std::shared_ptr<rmw_request_id_t> GenericClient::create_request_header() {
    return std::shared_ptr<rmw_request_id_t>(new rmw_request_id_t);
}

void GenericClient::handle_response(std::shared_ptr<rmw_request_id_t> request_header,
                                    std::shared_ptr<void> response) {
    std::unique_lock<std::mutex> lock(pending_requests_mutex_);
    int64_t sequence_number = request_header->sequence_number;

    auto serResponse = std::make_shared<rclcpp::SerializedMessage>();
    rmw_ret_t result = rmw_serialize(response.get(), _responseTypeSupportHdl,
                                     &serResponse->get_rcl_serialized_message());
    if (result != RMW_RET_OK) {
        RCLCPP_ERROR(rclcpp::get_node_logger(node_handle_.get()).get_child("rclcpp"),
                     "Failed to serialize service request. Ignoring...");
        return;
    }

    // TODO(esteve) this should throw instead since it is not expected to happen in the first place
    if (this->pending_requests_.count(sequence_number) == 0) {
        RCUTILS_LOG_ERROR("ServiceClient received invalid sequence number. Ignoring...");
        return;
    }
    auto tuple = this->pending_requests_[sequence_number];
    auto call_promise = std::get<0>(tuple);
    auto callback = std::get<1>(tuple);
    auto future = std::get<2>(tuple);
    this->pending_requests_.erase(sequence_number);
    // Unlock here to allow the service to be called recursively from one of its callbacks.
    lock.unlock();

    call_promise->set_value(serResponse);
    callback(future);
}

GenericClient::SharedFuture GenericClient::async_send_request(SharedRequest request) {
    return async_send_request(request, [](SharedFuture) {});
}

GenericClient::SharedFuture GenericClient::async_send_request(SharedRequest request,
                                                              CallbackType&& cb) {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    int64_t sequence_number;

    auto srv_members = static_cast<const ServiceMembers*>(_typeIntrospectionHdl->data);
    auto rosMsg = allocate_message(srv_members->request_members_);

    const rmw_serialized_message_t* rclRequest = &request->get_rcl_serialized_message();

    rcl_ret_t result;
    result = rmw_deserialize(rclRequest, _requestTypeSupportHdl, rosMsg.get());
    if (RCL_RET_OK != result) {
        rclcpp::exceptions::throw_from_rcl_error(result, "Failed to desirialize request");
    }

    result = rcl_send_request(get_client_handle().get(), rosMsg.get(), &sequence_number);
    if (RCL_RET_OK != result) {
        rclcpp::exceptions::throw_from_rcl_error(result, "Failed to send request");
    }

    SharedPromise call_promise = std::make_shared<Promise>();
    SharedFuture f(call_promise->get_future());
    pending_requests_[sequence_number] =
        std::make_tuple(call_promise, std::forward<CallbackType>(cb), f);
    return f;
}

}  // namespace dtnproxy::ros

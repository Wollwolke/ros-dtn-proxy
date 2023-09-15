#pragma once

#include <future>
#include <rclcpp/client.hpp>
#include <rclcpp/serialized_message.hpp>

#include "ros/type_helper.hpp"

// Based on
// https://github.com/foxglove/ros-foxglove-bridge/blob/f4b2f68cd/ros2_foxglove_bridge/include/foxglove_bridge/generic_client.hpp

namespace dtnproxy::ros {

class GenericClient : public rclcpp::ClientBase, private ServiceTypeSupport {
private:
    RCLCPP_DISABLE_COPY(GenericClient)

    using SharedRequest = std::shared_ptr<rclcpp::SerializedMessage>;
    using SharedResponse = std::shared_ptr<rclcpp::SerializedMessage>;
    using Promise = std::promise<SharedResponse>;
    using SharedPromise = std::shared_ptr<Promise>;
    using SharedFuture = std::shared_future<SharedResponse>;
    using CallbackType = std::function<void(SharedFuture)>;

    void initTypeHandles(const std::string& serviceType);

    std::map<int64_t, std::tuple<SharedPromise, CallbackType, SharedFuture>> pending_requests_;
    std::mutex pending_requests_mutex_;

public:
    RCLCPP_SMART_PTR_DEFINITIONS(GenericClient)

    GenericClient(rclcpp::node_interfaces::NodeBaseInterface* nodeBase,
                  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr nodeGraph,
                  const std::string& serviceName, const std::string& serviceType,
                  rcl_client_options_t& clientOptions);
    virtual ~GenericClient() {}

    std::shared_ptr<void> create_response() override;
    std::shared_ptr<rmw_request_id_t> create_request_header() override;
    void handle_response(std::shared_ptr<rmw_request_id_t> requestHeader,
                         std::shared_ptr<void> response) override;
    SharedFuture async_send_request(SharedRequest request);
    SharedFuture async_send_request(SharedRequest request, CallbackType&& cb);
};

}  // namespace dtnproxy::ros

#pragma once
#include <rclcpp/node.hpp>
#include <string>

#include "ros/type_helper.hpp"

namespace dtnproxy::ros {

class GenericService : public rclcpp::ServiceBase,
                       public std::enable_shared_from_this<GenericService>,
                       private ServiceTypeSupport {
private:
    RCLCPP_DISABLE_COPY(GenericService)
    using CallbackType = std::function<void(std::shared_ptr<rmw_request_id_t>,
                                            std::shared_ptr<rclcpp::SerializedMessage>)>;

    void initTypeHandles(const std::string &serviceType);

    CallbackType callback;

public:
    RCLCPP_SMART_PTR_DEFINITIONS(GenericService)

    GenericService(std::shared_ptr<rcl_node_t> nodeHandle, const std::string &serviceName,
                   const std::string &serviceType, CallbackType callback,
                   rcl_service_options_t &serviceOptions);
    virtual ~GenericService() {}

    std::shared_ptr<void> create_request() override;
    std::shared_ptr<rmw_request_id_t> create_request_header() override;

    void handle_request(std::shared_ptr<rmw_request_id_t> requestHeader,
                        std::shared_ptr<void> request) override;

    bool take_request(std::shared_ptr<void> requestOut, rmw_request_id_t &requestIdOut);

    void send_response(rmw_request_id_t &requestId,
                       std::shared_ptr<rclcpp::SerializedMessage> response);
};

}  // namespace dtnproxy::ros

#ifndef MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_
#define MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/go_to_loading.hpp"  // Custom service from Checkpoint 9

namespace my_components {

class AttachServer : public rclcpp::Node {
public:
    explicit AttachServer(const rclcpp::NodeOptions & options);

private:
    void handle_approach_shelf(
        const std::shared_ptr<custom_interfaces::srv::GoToLoading::Request> request,
        std::shared_ptr<custom_interfaces::srv::GoToLoading::Response> response);

    rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr service_;
};

}  // namespace my_components

#endif  // MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_

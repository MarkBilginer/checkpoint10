#include "my_components/attach_server_component.hpp"

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions & options)
    : Node("attach_server", options) {

    // Define the service for final approach to the shelf
    service_ = create_service<custom_interfaces::srv::GoToLoading>(
        "/approach_shelf", std::bind(
            &AttachServer::handle_approach_shelf, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Service /approach_shelf is ready.");
}

void AttachServer::handle_approach_shelf(
    const std::shared_ptr<custom_interfaces::srv::GoToLoading::Request> request,
    std::shared_ptr<custom_interfaces::srv::GoToLoading::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Final approach to the shelf is initiated.");

    // Simulate attaching to the shelf
    response->complete = true;
    RCLCPP_INFO(this->get_logger(), "Shelf attachment completed.");
}

}  // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)

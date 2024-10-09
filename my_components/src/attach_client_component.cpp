#include "my_components/attach_client_component.hpp"

using namespace std::chrono_literals;
using ServiceResponseFuture = rclcpp::Client<GoToLoading>::SharedFuture;

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("attach_client", options) {
  // Create a client that will call the /approach_shelf service
  client_ = this->create_client<GoToLoading>("/approach_shelf");

  call_approach_service();
}

void AttachClient::call_approach_service() {
  RCLCPP_INFO(this->get_logger(), "Calling approach service...");

  // Wait for the service to become available
  if (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available after waiting");
    return;
  }

  // Create a request
  auto request = std::make_shared<GoToLoading::Request>();
  request->attach_to_shelf = true;

  // Define a callback to handle the service response
  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    if (result->complete) {
      RCLCPP_INFO(this->get_logger(), "Final approach completed successfully.");
      rclcpp::shutdown(); // Optionally shut down the node if the task is
                          // complete
    } else {
      RCLCPP_ERROR(this->get_logger(), "Final approach failed.");
    }
  };

  // Send the service request asynchronously
  auto future_result =
      client_->async_send_request(request, response_received_callback);
}

} // namespace my_components

// Register this component with the class loader, so it can be dynamically
// loaded
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)

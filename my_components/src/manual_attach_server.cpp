/**
This keeps your AttachServer component definition and your manual composition
logic separate, adhering to modularity and separation of concerns. If someone
wants to use your component in runtime composition, they can still do so via the
original attach_server_component.cpp file, while others can opt for manual
composition via the manual_attach_server.cpp file.
**/

#include "my_components/attach_server_component.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Create an instance of AttachServer
  auto attach_server_node =
      std::make_shared<my_components::AttachServer>(rclcpp::NodeOptions());

  // Add it to the executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(attach_server_node);

  // Spin the executor
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

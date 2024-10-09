#ifndef MY_COMPONENTS__ATTACH_CLIENT_COMPONENT_HPP_
#define MY_COMPONENTS__ATTACH_CLIENT_COMPONENT_HPP_

#include "custom_interfaces/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

using GoToLoading = custom_interfaces::srv::GoToLoading;

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

protected:
  void call_approach_service();

private:
  rclcpp::Client<GoToLoading>::SharedPtr client_;
};

} // namespace my_components

#endif // MY_COMPONENTS__ATTACH_CLIENT_COMPONENT_HPP_

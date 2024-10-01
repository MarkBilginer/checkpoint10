#include "custom_interfaces/srv/go_to_loading.hpp" // Include the custom service
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

using GoToLoading = custom_interfaces::srv::GoToLoading;

class PreApproachV2 : public rclcpp::Node {
public:
  PreApproachV2() : Node("pre_approach_v2") {
    // Parameters
    this->declare_parameter("obstacle", 0.3);
    this->declare_parameter("degrees", -90);
    this->declare_parameter("final_approach", false);
    this->get_parameter("obstacle", obstacle_);
    this->get_parameter("degrees", degrees_);
    this->get_parameter("final_approach", final_approach_);

    // Convert degrees to an integer (if needed)
    degrees_ = static_cast<int>(degrees_);
    initial_yaw_ = 0.0;

    // Create callback groups
    callback_group_odom_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_scan_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_timer_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_group_final_approach_service_client_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Publishers and subscribers
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);

    // Create odom and scan subscriptions and assign them to callback groups
    auto odom_sub_opt = rclcpp::SubscriptionOptions();
    odom_sub_opt.callback_group = callback_group_odom_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PreApproachV2::odom_callback, this, std::placeholders::_1),
        odom_sub_opt);

    auto scan_sub_opt = rclcpp::SubscriptionOptions();
    scan_sub_opt.callback_group = callback_group_scan_;
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PreApproachV2::scan_callback, this, std::placeholders::_1),
        scan_sub_opt);

    rotate_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // Timer period
        std::bind(&PreApproachV2::rotate_robot_callback, this),
        callback_group_timer_);

    // Initialize variables
    moving_forward_ = true;
    rotation_complete_ = false;
    rotating_ = false;

    // Create a client for the final approach service
    final_approach_client_ = this->create_client<GoToLoading>(
        "/approach_shelf", rmw_qos_profile_default,
        callback_group_final_approach_service_client_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!rotation_complete_) {
      int center_index =
          msg->ranges.size() / 2; // Center corresponds to the robot's front
      int range_window = 10;      // A small window around the front
      bool obstacle_detected = false;

      // Check if any of the ranges in the front window detect an obstacle
      for (int i = center_index - range_window; i < center_index + range_window;
           ++i) {
        if (msg->ranges[i] < obstacle_) {
          obstacle_detected = true;
          break;
        }
      }

      // If moving forward and an obstacle is detected, stop and rotate
      if (moving_forward_ && obstacle_detected) {
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle detected. Stopping the robot.");
        stop_moving();
        start_rotation(); // Begin rotation when the robot stops due to an
                          // obstacle
      } else if (moving_forward_) {
        // Keep moving forward
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.4; // Move forward at 0.4 m/s
        vel_pub_->publish(twist_msg);

        // Log forward movement
        RCLCPP_INFO(this->get_logger(), "Moving forward at 0.4 m/s.");
      }
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract orientation quaternion from odom message
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    // Convert quaternion to Euler angles
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_yaw_ = yaw;

    // If the robot is rotating, check if the target yaw has been reached
    if (rotating_) {
      // Normalize both current and target yaws
      current_yaw_ = normalize_angle(current_yaw_);
      // Log current yaw value
      RCLCPP_INFO(this->get_logger(), "Current yaw: %f", current_yaw_);
      double yaw_difference = normalize_angle(current_yaw_ - target_yaw_);

      RCLCPP_INFO(this->get_logger(), "Yaw Difference: %f", yaw_difference);

      if (std::abs(yaw_difference) < 0.05) {
        RCLCPP_INFO(this->get_logger(),
                    "Target yaw reached. Stopping rotation.");
        stop_rotation();
        // Check if final_approach_ is true before calling the service
        call_approach_service(); // Call the final approach service only if
                                 // final_approach is true
      } else {
        RCLCPP_INFO(this->get_logger(), "Rotating... Yaw difference: %f",
                    yaw_difference);
      }
    }
  }

  void call_approach_service() {
    RCLCPP_INFO(this->get_logger(), "Calling approach service...");

    auto request = std::make_shared<GoToLoading::Request>();
    request->attach_to_shelf =
        final_approach_; // Use the final_approach_ parameter
    RCLCPP_INFO(this->get_logger(), "request->attach_to_shelf: %d",
                request->attach_to_shelf);

    // Wait for the service to become available
    while (!final_approach_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for service /approach_shelf...");
    }

    // Send the service request asynchronously
    auto result_future = final_approach_client_->async_send_request(request);

    // Spin to wait for the future to complete
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto result = result_future.get();
      if (result->complete) {
        RCLCPP_INFO(this->get_logger(),
                    "Final approach completed successfully.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Final approach failed.");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    }

    // Shutdown the node after the service is complete
    RCLCPP_INFO(this->get_logger(), "Shutting down PreApproachV2 node.");
    rclcpp::shutdown(); // Shutdown the node
  }

  void stop_moving() {
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.0;
    vel_pub_->publish(twist_msg);

    moving_forward_ = false;

    // Log stopping
    RCLCPP_INFO(this->get_logger(), "Robot stopped moving.");
  }

  void start_rotation() {
    rotating_ = true;
    initial_yaw_ = current_yaw_; // Record the initial yaw

    // Convert degrees to radians and calculate the target yaw
    target_yaw_ = normalize_angle(initial_yaw_ + degrees_ * (M_PI / 180.0));

    // Log the start of rotation
    RCLCPP_INFO(this->get_logger(), "Started rotating by %f radians.",
                initial_yaw_);
  }

  void rotate_robot_callback() {
    if (rotating_) {
      // Publish angular velocity to rotate the robot
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.angular.z =
          (degrees_ > 0) ? 0.5 : -0.5; // Rotate clockwise or counterclockwise
      vel_pub_->publish(twist_msg);

      // Log angular velocity
      RCLCPP_INFO(this->get_logger(), "Angular velocity: %f",
                  twist_msg.angular.z);
    }
  }

  void stop_rotation() {
    // Stop the robot's rotation
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.angular.z = 0.0;
    vel_pub_->publish(twist_msg);

    rotating_ = false;
    rotation_complete_ = true;

    // Log stopping the rotation
    RCLCPP_INFO(this->get_logger(),
                "Rotation complete. Robot stopped rotating.");
  }

  // Helper function to normalize angles to the range (-pi, pi)
  double normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr rotate_timer_;

  rclcpp::Client<GoToLoading>::SharedPtr final_approach_client_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr callback_group_odom_;
  rclcpp::CallbackGroup::SharedPtr callback_group_scan_;
  rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
  rclcpp::CallbackGroup::SharedPtr
      callback_group_final_approach_service_client_;

  bool moving_forward_;
  bool rotation_complete_;
  bool rotating_;
  double initial_yaw_;
  double current_yaw_;
  double target_yaw_;

  // Parameters
  double obstacle_;     // Obstacle distance threshold
  int degrees_;         // Rotation in degrees (input by user)
  bool final_approach_; // Should final_approach be performed?
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<PreApproachV2>();

  // Create a multi-threaded executor in the main function
  rclcpp::executors::MultiThreadedExecutor executor;

  // Add the node to the executor
  executor.add_node(node);

  // Spin the executor, which will handle the callbacks in multiple threads
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

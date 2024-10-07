#include "my_components/pre_approach_component.hpp"

namespace my_components {

PreApproach::PreApproach(const rclcpp::NodeOptions &options)
    : Node("pre_approach", options) {
  // Initialize variables
  moving_forward_ = true;
  rotation_complete_ = false;
  rotating_ = false;

  // Create publishers and subscribers
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

  // Create subscriptions
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

  rotate_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PreApproach::rotate_robot_callback, this));
}

void PreApproach::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
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
      RCLCPP_INFO(this->get_logger(), "Obstacle detected. Stopping the robot.");
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

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Extract orientation quaternion from odom message
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  // Convert quaternion to Euler angles
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_yaw_ = yaw;

  // Log current yaw value
  RCLCPP_INFO(this->get_logger(), "Current yaw: %f", current_yaw_);

  // If the robot is rotating, check if the target yaw has been reached
  if (rotating_) {
    // Normalize both current and target yaws
    current_yaw_ = normalize_angle(current_yaw_);
    double yaw_difference = normalize_angle(current_yaw_ - target_yaw_);

    RCLCPP_INFO(this->get_logger(), "Yaw Difference: %f", yaw_difference);

    if (std::abs(yaw_difference) < 0.05) {
      RCLCPP_INFO(this->get_logger(), "Target yaw reached. Stopping rotation.");
      stop_rotation();
    } else {
      RCLCPP_INFO(this->get_logger(), "Rotating... Yaw difference: %f",
                  yaw_difference);
    }
  }
}

void PreApproach::stop_moving() {
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = 0.0;
  vel_pub_->publish(twist_msg);

  moving_forward_ = false;

  // Log stopping
  RCLCPP_INFO(this->get_logger(), "Robot stopped moving.");
}

void PreApproach::start_rotation() {
  rotating_ = true;
  initial_yaw_ = current_yaw_; // Record the initial yaw

  // Convert degrees to radians and calculate the target yaw
  target_yaw_ = normalize_angle(initial_yaw_ + degrees_ * (M_PI / 180.0));

  // Log the start of rotation
  RCLCPP_INFO(this->get_logger(), "Started rotating by %f radians.",
              initial_yaw_);
}

void PreApproach::rotate_robot_callback() {
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

void PreApproach::stop_rotation() {
  // Stop the robot's rotation
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.angular.z = 0.0;
  vel_pub_->publish(twist_msg);

  rotating_ = false;
  rotation_complete_ = true;

  // Log stopping the rotation
  RCLCPP_INFO(this->get_logger(), "Rotation complete. Robot stopped rotating.");

  rclcpp::shutdown(); // This will gracefully shutdown the node
}

// Helper function to normalize angles to the range (-pi, pi)
double PreApproach::normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)
#ifndef MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_
#define MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_

#include "my_components/visibility_control.h"

// ROS2 core functionality for creating and managing nodes
#include "rclcpp/rclcpp.hpp"

// Custom service type for handling the approach to the shelf
#include "custom_interfaces/srv/go_to_loading.hpp"

// Message types for handling transformations and robot movement
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Message types for subscribing to sensor and odometry data
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp" // Include Empty message type for /elevator_up
#include "std_msgs/msg/string.hpp" // Include String message type for /elevator_up

// tf2 libraries for handling transforms, broadcasting and listening to frames
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

// tf2 libraries for geometry and quaternion operations
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ standard libraries for common functionalities
#include <cmath>   // For mathematical operations (sin, cos, etc.)
#include <iomanip> // For formatted output
#include <unordered_map> // For efficient data storage and lookup (scan data processing)

using GoToLoading = custom_interfaces::srv::GoToLoading;

namespace my_components {

class AttachServer : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachServer(const rclcpp::NodeOptions &options);

private:
  void
  handle_approach_shelf(const std::shared_ptr<GoToLoading::Request> request,
                        std::shared_ptr<GoToLoading::Response> response);

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void leg_detection_timer_callback();
  void process_scan_data();
  void process_odom_data();
  void publish_static_cart_transform();
  void publish_cart_transform();
  void stop_robot();
  void move_forward_by_distance(double distance);
  void move_timer_callback();
  void lift_shelf();

  rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      elevator_up_pub_; // Publisher for /elevator_up

  rclcpp::TimerBase::SharedPtr scan_processing_timer_;
  rclcpp::TimerBase::SharedPtr odom_processing_timer_;
  rclcpp::TimerBase::SharedPtr cart_transform_timer_;
  rclcpp::TimerBase::SharedPtr move_timer_;
  rclcpp::TimerBase::SharedPtr leg_detection_timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // Store the latest scan message
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;

  // Publisher for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;

  // tf2 Buffer and Listener for transformations
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  int attempts_left_;                               // Now a member variable
  std::shared_ptr<GoToLoading::Response> response_; // Store response

  // Variables to store detected leg information
  bool legs_detected_ = false;
  double midpoint_angle_;
  double midpoint_distance_;

  // Declare member variables here
  bool final_approach_;
  bool cart_published_;
  bool start_position_received_;

  double start_x_;
  double start_y_;
  double start_yaw_; // Yaw when the movement starts

  double current_x_;
  double current_y_;
  double current_yaw_; // Current yaw from odometry

  double distance_traveled_;
  double target_distance_;
};

} // namespace my_components

#endif // MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_

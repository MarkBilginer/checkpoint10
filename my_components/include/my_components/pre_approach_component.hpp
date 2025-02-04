#ifndef MY_COMPONENTS_PRE_APPROACH_COMPONENT_HPP
#define MY_COMPONENTS_PRE_APPROACH_COMPONENT_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

namespace my_components {

class PreApproach : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit PreApproach(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void process_scan_data();
  void process_odom_data();

  void stop_moving();
  void start_rotation();
  void rotate_robot_callback();
  void stop_rotation();
  double normalize_angle(double angle);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr scan_processing_timer_;
  rclcpp::TimerBase::SharedPtr odom_processing_timer_;
  rclcpp::TimerBase::SharedPtr rotate_timer_;

  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;

  bool moving_forward_;
  bool rotation_complete_;
  bool rotating_;
  double initial_yaw_;
  double current_yaw_;
  double target_yaw_;

  // Hardcoded parameters
  const double obstacle_ = 0.45; // Obstacle distance threshold
  const int degrees_ = -90;     // Rotation in degrees
};

} // namespace my_components

#endif // MY_COMPONENTS_PRE_APPROACH_COMPONENT_HPP

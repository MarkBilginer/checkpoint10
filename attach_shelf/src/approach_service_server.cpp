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

class ApproachService : public rclcpp::Node {
public:
  ApproachService() : Node("approach_service_server") {

    // Initialize variables
    moving_ = false;
    cart_published_ = false;
    lift_shelf_ = false;
    shelf_lifted_ = false;
    start_position_received_ = false;
    distance_traveled_ = 0.0;
    target_distance_ = 0.0;

    // Create the service
    service_ = this->create_service<GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachService::handle_service, this, std::placeholders::_1,
                  std::placeholders::_2));

    // Log that the service has been created
    RCLCPP_INFO(this->get_logger(),
                "Service /approach_shelf is up and running.");

    // Create the LaserScan subscriber to detect the legs
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ApproachService::scan_callback, this,
                  std::placeholders::_1));

    // Create a publisher for /elevator_up
    elevator_up_pub_ =
        this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

    // Log the creation of the scan subscriber
    RCLCPP_INFO(this->get_logger(), "Subscriber for /scan topic created.");

    // Odometry subscriber for robot movement control (initially inactive)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ApproachService::odom_callback, this,
                  std::placeholders::_1));
    // Log the creation of the odom subscriber
    RCLCPP_INFO(this->get_logger(), "Subscriber for /odom topic created.");

    // Set up the transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    RCLCPP_INFO(this->get_logger(), "Transform broadcaster initialized.");

    // Timer to process scan data every 500ms
    scan_processing_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ApproachService::process_scan_data, this));

    // Timer to continuously publish the cart_frame transform
    cart_transform_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), // Adjust the frequency as needed
        std::bind(&ApproachService::publish_cart_transform, this));

    // Initially, stop the timer until the service is called
    cart_transform_timer_->cancel();

    move_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // Timer callback every 100 ms
        std::bind(&ApproachService::move_timer_callback, this));
    move_timer_->cancel();

    // Create a publisher for sending velocity commands
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);

    // Initialize tf2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Approach Service Server created.");
  }

private:
  void handle_service(const std::shared_ptr<GoToLoading::Request> request,
                      std::shared_ptr<GoToLoading::Response> response) {

    const bool attach_to_shelf = request->attach_to_shelf;
    RCLCPP_INFO(this->get_logger(), "attach_to_shelf: %d", attach_to_shelf);
    final_approach_ = attach_to_shelf;
    if (attach_to_shelf) {
      RCLCPP_INFO(this->get_logger(), "Performing final approach...");
      if (legs_detected_) {
        RCLCPP_INFO(
            this->get_logger(),
            "Legs detected, proceeding to publish transform and move robot.");
        // Start publishing the cart transform periodically
        cart_transform_timer_->reset();
        // lift_shelf();
        response->complete = true;
        RCLCPP_INFO(this->get_logger(),
                    "Final approach successful. Response set to True.");
      } else {
        RCLCPP_WARN(this->get_logger(), "Could not detect both shelf legs.");
        response->complete = false;
      }
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Only publishing transform, no final approach.");
      cart_transform_timer_->reset();
      response->complete = true;
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Just store the latest scan message without processing it
    last_scan_ = msg;
  }

  void process_scan_data() {
    if (!last_scan_) {
      RCLCPP_WARN(this->get_logger(), "No scan data available yet.");
      return;
    }
    std::vector<std::vector<int>> clusters;
    std::vector<int> current_cluster;
    const int min_cluster_size = 5; // Minimum cluster size to filter noise
    const double expected_leg_distance =
        0.6; // Expected distance between shelf legs in meters
    const double tolerance =
        0.15; // Allowable deviation from expected leg distance

    // Create a map to store the count of each unique range value
    std::unordered_map<float, int> intensity_counts;

    // Iterate through the range values and populate the map
    for (size_t i = 0; i < last_scan_->intensities.size(); ++i) {
      float intensity_value = last_scan_->intensities[i];

      // Increase the count of the current range value in the map
      intensity_counts[intensity_value]++;
    }

    // Print the total number of indexes
    RCLCPP_INFO(this->get_logger(), "Total number of indices: %lu",
                last_scan_->intensities.size());

    // Print the aggregated range values along with their counts
    RCLCPP_INFO(this->get_logger(),
                "Aggregated intensity values with their counts:");

    for (const auto &entry : intensity_counts) {
      RCLCPP_INFO(this->get_logger(), "Range value: %f, Count: %d", entry.first,
                  entry.second);
    }

    // Detect clusters of consecutive beams with intensities over 8000
    RCLCPP_INFO(this->get_logger(),
                "Processing laser scan data to detect legs...");
    for (size_t i = 0; i < last_scan_->intensities.size(); ++i) {
      if (last_scan_->intensities[i] >= 8000) {
        current_cluster.push_back(i);
      } else if (!current_cluster.empty()) {
        clusters.push_back(current_cluster);
        current_cluster.clear();
      }
    }

    // Add the last cluster if it exists
    if (!current_cluster.empty()) {
      clusters.push_back(current_cluster);
    }

    // Log the number of detected clusters
    RCLCPP_INFO(this->get_logger(), "Number of clusters detected: %lu",
                clusters.size());

    // Ensure we have detected two clusters (legs) with minimum size
    if (clusters.size() >= 2 && clusters[0].size() >= min_cluster_size &&
        clusters[1].size() >= min_cluster_size) {
      // Calculate the midpoints for the two largest clusters (legs)
      auto first_leg_cluster = clusters[0];
      auto second_leg_cluster = clusters[1];
      // Log the number of detected clusters
      RCLCPP_INFO(this->get_logger(), "first_leg_cluster detected: %lu",
                  first_leg_cluster.size());

      for (int intensity : first_leg_cluster) {
        RCLCPP_INFO(this->get_logger(), "%d", intensity);
      }

      RCLCPP_INFO(this->get_logger(), "second_leg_cluster detected: %lu",
                  second_leg_cluster.size());

      for (int intensity : second_leg_cluster) {
        RCLCPP_INFO(this->get_logger(), "%d", intensity);
      }

      // Initialize variables to sum the x and y coordinates for each leg
      double first_leg_x_sum = 0.0;
      double first_leg_y_sum = 0.0;

      double second_leg_x_sum = 0.0;
      double second_leg_y_sum = 0.0;

      // Calculate the sum of Cartesian coordinates for the first leg
      for (int idx : first_leg_cluster) {
        RCLCPP_INFO(this->get_logger(), "Index: %d, Range: %f", idx,
                    last_scan_->ranges[idx]);
        double angle =
            last_scan_->angle_min + idx * last_scan_->angle_increment;
        double distance = last_scan_->ranges[idx];

        first_leg_x_sum += distance * std::cos(angle);
        first_leg_y_sum += distance * std::sin(angle);
      }

      // Calculate the sum of Cartesian coordinates for the second leg
      for (int idx : second_leg_cluster) {
        RCLCPP_INFO(this->get_logger(), "Index: %d, Range: %f", idx,
                    last_scan_->ranges[idx]);
        double angle =
            last_scan_->angle_min + idx * last_scan_->angle_increment;
        double distance = last_scan_->ranges[idx];

        second_leg_x_sum += distance * std::cos(angle);
        second_leg_y_sum += distance * std::sin(angle);
      }

      // Compute the average x and y coordinates for each leg (centroid)
      double first_leg_x_avg = first_leg_x_sum / first_leg_cluster.size();
      double first_leg_y_avg = first_leg_y_sum / first_leg_cluster.size();

      double second_leg_x_avg = second_leg_x_sum / second_leg_cluster.size();
      double second_leg_y_avg = second_leg_y_sum / second_leg_cluster.size();

      // Log centroids
      RCLCPP_INFO(this->get_logger(), "First leg centroid: x = %f, y = %f",
                  first_leg_x_avg, first_leg_y_avg);
      RCLCPP_INFO(this->get_logger(), "Second leg centroid: x = %f, y = %f",
                  second_leg_x_avg, second_leg_y_avg);

      // Calculate the distance between the two legs based on centroids
      double distance_between_legs =
          std::sqrt(std::pow((second_leg_x_avg - first_leg_x_avg), 2) +
                    std::pow((second_leg_y_avg - first_leg_y_avg), 2));

      // Log the calculated distance between the two legs
      RCLCPP_INFO(this->get_logger(),
                  "Calculated distance between legs: %f meters",
                  distance_between_legs);

      // Check if the distance between the legs is within the expected range
      if (std::abs(distance_between_legs - expected_leg_distance) <=
          tolerance) {
        legs_detected_ = true;
        // scan_processing_timer_->cancel();

        // Calculate the midpoint between the two legs
        double midpoint_x = (first_leg_x_avg + second_leg_x_avg) / 2.0;
        double midpoint_y = (first_leg_y_avg + second_leg_y_avg) / 2.0;

        // Convert midpoint back to polar coordinates
        midpoint_distance_ =
            std::sqrt(midpoint_x * midpoint_x + midpoint_y * midpoint_y);
        midpoint_angle_ = std::atan2(midpoint_y, midpoint_x);

        RCLCPP_INFO(
            this->get_logger(),
            "Both legs detected. Midpoint distance: %f, Midpoint angle: %f",
            midpoint_distance_, midpoint_angle_);
      } else {
        legs_detected_ = false;
        RCLCPP_WARN(this->get_logger(),
                    "Legs detected, but the distance between them is "
                    "incorrect. Expected: %f, Got: %f",
                    expected_leg_distance, distance_between_legs);
      }
    } else {
      legs_detected_ = false;
      RCLCPP_WARN(this->get_logger(),
                  "Could not detect two valid legs with minimum size.");
    }
  }

  void publish_cart_transform() {
    if (!legs_detected_) {
      RCLCPP_WARN(this->get_logger(),
                  "Cannot publish cart transform, legs not detected.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Publishing cart_frame transform...");

    // Create the transform and publish it
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id =
        "robot_front_laser_base_link"; // Parent frame is robot_base_link
    transformStamped.child_frame_id = "cart_frame"; // The new cart_frame
    transformStamped.transform.translation.x =
        midpoint_distance_ * cos(midpoint_angle_);
    transformStamped.transform.translation.y =
        midpoint_distance_ * sin(midpoint_angle_);
    transformStamped.transform.translation.z = 0.0;

    // Identity quaternion (no rotation)
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transformStamped);

    RCLCPP_INFO(this->get_logger(), "Published cart_frame transform.");
    RCLCPP_INFO(this->get_logger(),
                "Publishing cart_frame transform... X: %f, Y: %f",
                midpoint_distance_ * cos(midpoint_angle_),
                midpoint_distance_ * sin(midpoint_angle_));

    cart_published_ = true; // Set flag to true when cart_frame is published
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    if (final_approach_) {

      current_x_ = msg->pose.pose.position.x;
      current_y_ = msg->pose.pose.position.y;

      // Extract yaw (rotation around the z-axis) from the quaternion
      tf2::Quaternion q(
          msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      current_yaw_ = yaw; // Store the current yaw value

      // Optionally, log the odom data if needed for debugging
      RCLCPP_INFO(this->get_logger(), "Odom - X: %f, Y: %f, Yaw: %f",
                  current_x_, current_y_, current_yaw_);

      // Handle the case where the cart frame is being approached
      if (cart_published_) {

        // Move the robot towards the cart_frame using odom data
        geometry_msgs::msg::TransformStamped transformStamped;

        try {
          // Get the transform from the robot base_link to cart_frame
          transformStamped = tf_buffer_->lookupTransform(
              "robot_base_link", "cart_frame", tf2::TimePointZero,
              tf2::durationFromSec(0.2));
        } catch (tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
          return;
        }

        // Calculate the distance and angle to cart_frame
        double dx = transformStamped.transform.translation.x;
        double dy = transformStamped.transform.translation.y;
        double distance = sqrt(dx * dx + dy * dy);
        double angle_to_goal = atan2(dy, dx);

        RCLCPP_INFO(this->get_logger(),
                    "Distance to cart: %f, Angle to cart: %f", distance,
                    angle_to_goal);

        // Stop if the robot is close enough to the cart_frame
        if (distance < 0.1) {
          stop_robot();
          RCLCPP_INFO(this->get_logger(),
                      "Reached cart_frame! Stopping transform publishing.");
          scan_processing_timer_->cancel();
          cart_transform_timer_->cancel();

          // Move the robot forward by 30 cm
          move_forward_by_distance(0.5); // 0.3 meters = 30 cm

          return;
        }

        // Create a Twist message for velocity commands
        geometry_msgs::msg::Twist cmd_vel;

        // Set linear velocity proportional to distance
        cmd_vel.linear.x = std::min(0.3, distance * 0.5);
        cmd_vel.angular.z = 0.0;
        velocity_pub_->publish(cmd_vel);
      }
    }
  }

  // Function to stop the robot
  void stop_robot() {
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    velocity_pub_->publish(stop_msg);
    moving_ = false;
    RCLCPP_INFO(this->get_logger(), "Robot stopped.");
    RCLCPP_INFO(this->get_logger(),
                "Published stop command: linear.x = 0.0, angular.z = 0.0");
  }

  // Function to start moving the robot forward by a specified distance
  void move_forward_by_distance(double distance) {
    target_distance_ = distance;
    distance_traveled_ = 0.0; // Reset the traveled distance
    start_position_received_ =
        false; // Flag to indicate when the first odometry is received

    cart_published_ = false;
    // Start moving the robot
    moving_ = true;
    move_timer_->reset(); // start the timer for moving robot 30cm

    RCLCPP_INFO(this->get_logger(), "Starting to move forward for %f meters",
                distance);
  }

  void move_timer_callback() {

    if (moving_) {
      if (!start_position_received_) {
        // Capture the initial position once
        start_x_ = current_x_;
        start_y_ = current_y_;
        start_yaw_ = current_yaw_; // Capture the initial yaw
        start_position_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Start position and yaw received.");
        return;
      }

      // Calculate the distance traveled from the starting position
      double dx = current_x_ - start_x_;
      double dy = current_y_ - start_y_;
      distance_traveled_ = sqrt(dx * dx + dy * dy);

      RCLCPP_INFO(this->get_logger(), "dx: %f meters", dx);
      RCLCPP_INFO(this->get_logger(), "dy: %f meters", dy);

      // RCLCPP_INFO(this->get_logger(),
      //             "Distance traveled: %f meters,\n Target Distance: %f
      //             meters", distance_traveled_, target_distance_);

      // RCLCPP_INFO(this->get_logger(),
      //            "Distance traveled dx: %f meters,\n Target Distance: %f
      //           meters", dx, target_distance_);

      // Stop the robot if it has traveled the desired distance
      if (distance_traveled_ >= target_distance_) {
        moving_ = false;
        move_timer_
            ->cancel(); // Cancel the timer after reaching the target distance

        stop_robot();

        RCLCPP_INFO(this->get_logger(), "Reached target distance of %f meters",
                    target_distance_);

        lift_shelf_ = true;
        lift_shelf();

        return;
      }

      // Calculate the yaw drift (difference between the current yaw and the
      // starting yaw)
      double yaw_error = current_yaw_ - start_yaw_;

      // Normalize the yaw error to be within [-pi, pi]
      yaw_error = atan2(sin(yaw_error), cos(yaw_error));

      // Publish forward velocity command with yaw correction
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.1; // Forward velocity
      cmd_vel.angular.z = 0.0;
      // cmd_vel.angular.z =
      //   -yaw_error * 2.5; // Correct yaw drift (adjust 0.5 to tune
      //   response)

      velocity_pub_->publish(cmd_vel);
      RCLCPP_INFO(this->get_logger(),
                  "Continuing to move forward with yaw correction...");
    }
  }

  void lift_shelf() {
    // RCLCPP_INFO(this->get_logger(), "You can now move the elevator up!");
    //  Check if the robot is positioned correctly underneath the shelf
    if (!lift_shelf_) {
      RCLCPP_WARN(this->get_logger(),
                  "The robot is not underneath the shelf yet!");
      return;
    } else if (shelf_lifted_) {
      RCLCPP_WARN(this->get_logger(), "The shelf is already lifted!");
      return;
    }

    // Create and initialize a String message
    std_msgs::msg::String msg;
    msg.data = "lift_up"; // Same as the data you passed in the command line

    // Publish the message
    elevator_up_pub_->publish(msg);

    // Create an Empty message to publish
    // std_msgs::msg::Empty empty_msg;

    // Publish the Empty message to lift the elevator
    // elevator_up_pub_->publish(empty_msg);

    // RCLCPP_INFO(this->get_logger(), "Elevator is moving up to lift the
    // shelf.");
    // lift_shelf_ = false;
    // shelf_lifted_ = true;

    // Add a delay to allow time for the shelf to be lifted (optional, based
    // on your setup) rclcpp::sleep_for(std::chrono::seconds(3));

    // Cleanly shut down the ROS2 node after lifting the shelf
    // RCLCPP_INFO(this->get_logger(), "Shelf lifted. Shutting down...");
    RCLCPP_INFO(this->get_logger(), "The shelf has been lifted.");
    rclcpp::shutdown();
  }

  rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      elevator_up_pub_; // Publisher for /elevator_up

  rclcpp::TimerBase::SharedPtr scan_processing_timer_;
  rclcpp::TimerBase::SharedPtr cart_transform_timer_;
  rclcpp::TimerBase::SharedPtr move_timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Store the latest scan message
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  // Publisher for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;

  // tf2 Buffer and Listener for transformations
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Variables to store detected leg information
  bool legs_detected_ = false;
  double midpoint_angle_;
  double midpoint_distance_;

  // Declare member variables here
  bool moving_;
  bool cart_published_;
  bool final_approach_;
  bool start_position_received_;
  bool lift_shelf_;
  bool shelf_lifted_;

  double start_x_;
  double start_y_;
  double start_yaw_; // Yaw when the movement starts

  double current_x_;
  double current_y_;
  double current_yaw_; // Current yaw from odometry

  double distance_traveled_;
  double target_distance_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachService>();

  // Use single-threaded executor (no callback groups)
  rclcpp::spin(node);

  // No need to call rclcpp::shutdown() here since it's already called in
  // lift_shelf()
  return 0;
}

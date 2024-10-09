#include "my_components/attach_server_component.hpp"

using namespace std::chrono_literals;

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions &options)
    : Node("attach_server", options) {

  // Define the service for final approach to the shelf
  service_ = create_service<GoToLoading>(
      "/approach_shelf",
      std::bind(&AttachServer::handle_approach_shelf, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Service /approach_shelf is ready.");

  // Create the LaserScan subscriber to detect the legs
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&AttachServer::scan_callback, this, std::placeholders::_1));

  // Create a publisher for /elevator_up
  elevator_up_pub_ =
      this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

  // Log the creation of the scan subscriber
  RCLCPP_INFO(this->get_logger(), "Subscriber for /scan topic created.");

  // Odometry subscriber for robot movement control (initially inactive)
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&AttachServer::odom_callback, this, std::placeholders::_1));
  // Log the creation of the odom subscriber
  RCLCPP_INFO(this->get_logger(), "Subscriber for /odom topic created.");

  // Set up the transform broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  // Create the static transform broadcaster
  static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  RCLCPP_INFO(this->get_logger(), "Transform broadcaster initialized.");

  // Timer to process scan data every 500ms
  scan_processing_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&AttachServer::process_scan_data, this));

  // Timer to process odometry data every 100ms (disabled initially)
  odom_processing_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&AttachServer::process_odom_data, this));
  odom_processing_timer_->cancel();

  // Timer to continuously publish the cart_frame transform
  cart_transform_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), // Adjust the frequency as needed
      std::bind(&AttachServer::publish_static_cart_transform, this));

  // Initially, stop the timer until the service is called
  cart_transform_timer_->cancel();

  move_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), // Timer callback every 100 ms
      std::bind(&AttachServer::move_timer_callback, this));
  move_timer_->cancel();

  // Create a publisher for sending velocity commands
  velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

  // Initialize tf2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "Approach Service Server created.");
}

void AttachServer::handle_approach_shelf(
    const std::shared_ptr<GoToLoading::Request> request,
    std::shared_ptr<GoToLoading::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Final approach to the shelf is initiated.");

  final_approach_ = request->attach_to_shelf;
  response_ = response; // Store the response in the class member
  attempts_left_ = 20;  // Initialize attempts_left

  if (final_approach_) {
    RCLCPP_INFO(this->get_logger(), "Performing final approach...");

    leg_detection_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&AttachServer::leg_detection_timer_callback, this));
  } else {
    RCLCPP_INFO(this->get_logger(), "Final approach abandoned.");
    response->complete = false;
  }
}

void AttachServer::leg_detection_timer_callback() {
  if (attempts_left_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "Timeout: Legs could not be detected.");
    response_->complete = false;
    leg_detection_timer_->cancel();
    return;
  }

  if (legs_detected_) {
    RCLCPP_INFO(this->get_logger(),
                "Legs detected, proceeding with final approach.");
    odom_processing_timer_->reset(); // Start processing odom
    cart_transform_timer_->reset();  // Start publishing the cart transform
    leg_detection_timer_->cancel();  // Stop the timer once detected
    response_->complete = true;
  } else {
    RCLCPP_INFO(this->get_logger(), "Legs not detected yet, attempts left: %d",
                attempts_left_);
    --attempts_left_;
  }
}

void AttachServer::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Just store the latest scan message without processing it
  last_scan_ = msg;
}

void AttachServer::process_scan_data() {
  if (!last_scan_) {
    RCLCPP_WARN(this->get_logger(), "No scan data available yet.");
    return;
  }

  if (final_approach_) {

    std::vector<std::vector<int>> clusters;
    std::vector<int> current_cluster;
    const int min_cluster_size = 10; // Minimum cluster size to filter noise
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
    if (clusters.size() >= 2) {
      RCLCPP_INFO(this->get_logger(),
                  "Number of clusters in Cluster 0 detected: %lu",
                  clusters[0].size());
      RCLCPP_INFO(this->get_logger(),
                  "Number of clusters in Cluster 1 detected: %lu",
                  clusters[1].size());
    }

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
}

void AttachServer::publish_static_cart_transform() {
  if (!legs_detected_) {
    RCLCPP_WARN(this->get_logger(),
                "Cannot publish static cart transform, legs not detected.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Publishing static cart_frame transform...");

  geometry_msgs::msg::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = this->get_clock()->now();
  static_transformStamped.header.frame_id =
      "robot_front_laser_base_link";                     // Parent frame
  static_transformStamped.child_frame_id = "cart_frame"; // Child frame

  // Set the translation and rotation
  static_transformStamped.transform.translation.x =
      midpoint_distance_ * cos(midpoint_angle_);
  static_transformStamped.transform.translation.y =
      midpoint_distance_ * sin(midpoint_angle_);
  static_transformStamped.transform.translation.z = 0.0;

  static_transformStamped.transform.rotation.x = 0.0;
  static_transformStamped.transform.rotation.y = 0.0;
  static_transformStamped.transform.rotation.z = 0.0;
  static_transformStamped.transform.rotation.w = 1.0;

  // Publish the static transform
  static_tf_broadcaster_->sendTransform(static_transformStamped);

  RCLCPP_INFO(this->get_logger(), "Static cart_frame transform published.");

  cart_published_ = true; // Set flag to true when cart_frame is published
}

void AttachServer::publish_cart_transform() {
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

  RCLCPP_INFO(this->get_logger(), "Dynamic cart_frame transform published.");

  cart_published_ = true; // Set flag to true when cart_frame is published
}

void AttachServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Just store the latest odometry message without processing it immediately
  last_odom_ = msg;
}

void AttachServer::process_odom_data() {

  if (!last_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry data available yet.");
    return;
  }

  current_x_ = last_odom_->pose.pose.position.x;
  current_y_ = last_odom_->pose.pose.position.y;

  // Extract yaw (rotation around the z-axis) from the quaternion
  tf2::Quaternion q(
      last_odom_->pose.pose.orientation.x, last_odom_->pose.pose.orientation.y,
      last_odom_->pose.pose.orientation.z, last_odom_->pose.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_yaw_ = yaw; // Store the current yaw value

  // Optionally, log the odom data if needed for debugging
  RCLCPP_INFO(this->get_logger(), "Odom - X: %f, Y: %f, Yaw: %f", current_x_,
              current_y_, current_yaw_);

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

    RCLCPP_INFO(this->get_logger(), "Distance to cart: %f, Angle to cart: %f",
                distance, angle_to_goal);

    // Stop if the robot is close enough to the cart_frame
    if (distance < 0.1) {
      scan_processing_timer_->cancel();
      cart_transform_timer_->cancel();
      stop_robot();
      RCLCPP_INFO(this->get_logger(),
                  "Reached cart_frame! Stopping transform publishing.");

      // Move the robot forward by 30 cm
      move_forward_by_distance(0.45); // 0.3 meters = 30 cm

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

void AttachServer::stop_robot() {
  geometry_msgs::msg::Twist stop_msg;
  stop_msg.linear.x = 0.0;
  stop_msg.angular.z = 0.0;
  velocity_pub_->publish(stop_msg);
  RCLCPP_INFO(this->get_logger(), "Robot stopped.");
  RCLCPP_INFO(this->get_logger(),
              "Published stop command: linear.x = 0.0, angular.z = 0.0");
}

void AttachServer::move_forward_by_distance(double distance) {
  target_distance_ = distance;
  distance_traveled_ = 0.0; // Reset the traveled distance
  start_position_received_ =
      false; // Flag to indicate when the first odometry is received

  cart_published_ = false;
  // Start moving the robot
  move_timer_->reset(); // start the timer for moving robot 30cm

  RCLCPP_INFO(this->get_logger(), "Starting to move forward for %f meters",
              distance);
}

void AttachServer::move_timer_callback() {

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

  // Stop the robot if it has traveled the desired distance
  if (distance_traveled_ >= target_distance_) {
    // Cancel the timers after reaching the target distance
    odom_processing_timer_->cancel();
    move_timer_->cancel();

    stop_robot();

    RCLCPP_INFO(this->get_logger(), "Reached target distance of %f meters",
                target_distance_);

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

  velocity_pub_->publish(cmd_vel);
  RCLCPP_INFO(this->get_logger(),
              "Continuing to move forward with yaw correction...");
}

void AttachServer::lift_shelf() {

  // Create and initialize a String message
  std_msgs::msg::String msg;
  msg.data = "lift_up"; // Same as the data you passed in the command line

  // Publish the message
  elevator_up_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(),
              "The shelf has been lifted. Shutting down...");
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)

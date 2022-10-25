#include "convoy_ros/safety.hpp"

Safety::Safety() : Node("safety")
{
  // ROS parameters
  this->declare_parameter<std::string>("veh_ns", "");
  this->declare_parameter<float>("ttc_thresh", 1.0);

  this->declare_parameter<std::string>("brake_topic", "");
  this->declare_parameter<std::string>("brake_bool_topic", "");
  this->declare_parameter<std::string>("drive_topic", "");
  this->declare_parameter<std::string>("joy_topic", "");
  this->declare_parameter<std::string>("odom_topic", "");
  this->declare_parameter<std::string>("scan_topic", "");

  this->get_parameter("ttc_thresh", ttc_thresh_);
  this->get_parameter("veh_ns", veh_ns_);

  this->get_parameter("brake_bool_topic", brake_bool_topic_);
  this->get_parameter("drive_topic", drive_topic_);
  this->get_parameter("joy_topic", joy_topic_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("scan_topic", scan_topic_);

  odom_topic_ = veh_ns_ + "/" + odom_topic_;

	// publishers
  brake_bool_pub_ = this->create_publisher<Bool>(
      brake_bool_topic_, 1
  );
	drive_pub_ = this->create_publisher<AckermannDriveStamped>(
			drive_topic_, 1
	);

	// subscribers
  joy_sub_ = this->create_subscription<Joy>(
      joy_topic_, 1, std::bind(&Safety::joy_cb, this, _1)
  );
	odom_sub_ = this->create_subscription<Odometry>(
			odom_topic_, 1, std::bind(&Safety::odom_cb, this, _1)
	);
	scan_sub_ = this->create_subscription<LaserScan>(
			scan_topic_, 1, std::bind(&Safety::scan_cb, this, _1)
	);

  RCLCPP_INFO(this->get_logger(), "Initialized subscribers");

	// variables
	brake_msg_.drive.speed = 0.0;
}

// subscriber callbacks

void Safety::joy_cb(const Joy::SharedPtr msg)
{
	// R2 button will be used for braking
	// it's value is on msg.axes[5], and takes on values in range [-1, 1]
	// msg.axes[5] in (-1, 1] -> button not fully pressed -> no brake
	// msg.axes[5] == -1 -> button fully pressed -> apply brake
	if (msg->axes[5] == r2_min_) {
    brake_msg_.header.stamp = this->now();
    drive_pub_->publish(brake_msg_);
  }
}

void Safety::odom_cb(const Odometry::SharedPtr msg)
{
  speed_curr_ = msg->twist.twist.linear.x;
}

void Safety::scan_cb(const LaserScan::SharedPtr msg)
{
  std::vector<float> ttc_vec;
  ttc_vec = calc_ttc(msg);
  if (send_brake_cmd(ttc_vec)) {
    RCLCPP_INFO(this->get_logger(), "COLLISION IMMINENT: EMERGENCY BRAKING.");
    brake_msg_.header.stamp = this->now();
    drive_pub_->publish(brake_msg_);
    brake_bool_pub_->publish(true);
  }
}

// helper functions

// calculate time to collision based on the vehicle's current position and 
// the scan poins
std::vector<float> Safety::calc_ttc(const LaserScan::SharedPtr msg)
{
  std::vector<float> ttc_vec;
  float angle_curr = msg->angle_min;
  for (const auto& r : msg->ranges)
  {
    float ttc;
    float r_dot = -speed_curr_ * std::cos(angle_curr);
    float val = std::max(0.0f, -r_dot);
    if (val == 0.0f)
      ttc = std::numeric_limits<float>::infinity();
    else
      ttc = r / val;

    ttc_vec.push_back(ttc);

    angle_curr += msg->angle_increment;
  }
  return ttc_vec;
}

// determine whether or not we should send a brake command based on the 
// time to colllision threshold
bool Safety::send_brake_cmd(const std::vector<float> ttc_vec)
{
  float ttc = *std::min_element(ttc_vec.begin(), ttc_vec.end());
  if (ttc < ttc_thresh_ + 0.1)
    RCLCPP_INFO(this->get_logger(), "min ttc: '%f'", ttc);
  return ttc < ttc_thresh_;
}

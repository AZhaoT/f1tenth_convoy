#ifndef SAFETY__SAFETY_HPP_
#define SAFETY__SAFETY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"

using ackermann_msgs::msg::AckermannDriveStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Joy;
using sensor_msgs::msg::LaserScan;
using std_msgs::msg::Bool;
using std_msgs::msg::Header;

using std::placeholders::_1;

class Safety : public rclcpp::Node
{

public:

	Safety();

private:

  // ROS parameters
  float ttc_thresh_;
  std::string veh_ns_;

  std::string brake_bool_topic_;
  std::string drive_topic_;
  std::string joy_topic_;
  std::string odom_topic_;
  std::string scan_topic_;
  
	// publishers
  rclcpp::Publisher<Bool>::SharedPtr brake_bool_pub_;
	rclcpp::Publisher<AckermannDriveStamped>::SharedPtr drive_pub_;

	// subscribers
	rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
	rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;

	// subscriber callbacks
  void joy_cb(const Joy::SharedPtr msg);
  void odom_cb(const Odometry::SharedPtr msg);
  void scan_cb(const LaserScan::SharedPtr msg);

  // variables
	double r2_min_ = -1.0;  // joy_cb
	AckermannDriveStamped brake_msg_;  // joy_cb, scan_cb
  float speed_curr_;  // calc_ttc

  // helper functions
  std::vector<float> calc_ttc(const LaserScan::SharedPtr msg);
  bool send_brake_cmd(const std::vector<float> ttc_vec);

};

#endif  // SAFETY__SAFETY_HPP_

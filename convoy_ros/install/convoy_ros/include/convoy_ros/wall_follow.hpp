#ifndef WALLFOLLOW__WALLFOLLOW_HPP_
#define WALLFOLLOW__WALLFOLLOW_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"

using ackermann_msgs::msg::AckermannDriveStamped;
using sensor_msgs::msg::LaserScan;
using std_msgs::msg::Header;

using std::placeholders::_1;

class WallFollow : public rclcpp::Node
{

public:

  WallFollow();

private:

  // ROS parameters
  float kp_;
  float ki_;
  float kd_;
  float L_;  // lookahead distance
  float dist_angle_const_;
  float left_dist_des_;
  float right_dist_des_;
  float low_angle_speed_;
  float low_angle_;
  float med_angle_speed_;
  float med_angle_;
  float high_angle_speed_;
  float max_angle_;
  float theta_; // angle at which we get second wall range

  std::string drive_topic_;
  std::string scan_topic_;

  // publishers
  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr drive_pub_;

  // subscribers
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;

  // subscriber callbacks
  void scan_cb(const LaserScan::SharedPtr msg);

  // variables
  float prev_error_;
  float integral_;
  // wall following parameters
  float zero_angle_;
  float angle_range_;

  // helper functions
  float calc_error(float a, float b);
  float calc_speed(float steering_angle);
  float calc_steer_angle(float error);
  long get_angle_ind(const LaserScan::SharedPtr msg, float angle);
  float get_range(const LaserScan::SharedPtr msg, float angle);
  void pid_control(const LaserScan::SharedPtr msg);

};

#endif  // WALLFOLLOW__WALLFOLLOW_HPP_

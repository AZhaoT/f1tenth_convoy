#ifndef GAPFOLLOW__GAPFOLLOW_HPP_
#define GAPFOLLOW__GAPFOLLOW_HPP_

#include <algorithm>
#include <cmath>
#include <numeric>
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

class GapFollow : public rclcpp::Node
{

public:

  GapFollow();

private:

  // ROS parameters
  float car_radius_;
  float scan_thresh_m_;
  float disparity_thresh_m_;
  float cornering_thresh_m_;
  float low_angle_speed_;
  float low_angle_;
  float med_angle_speed_;
  float med_angle_;
  float high_angle_speed_;
  float max_angle_;
  
  std::string drive_topic_;
  std::string scan_topic_;

  // publishers
  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<LaserScan>::SharedPtr proc_scan_pub_;

  // subscribers
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;

  // subscriber callbacks
  void scan_cb(const LaserScan::SharedPtr msg);

  // variables
  int num_scans_;
  float angle_inc_;
  // ind variables for where the zero safety bubble currently is
  int safety_start_i_;
  int safety_end_i_;
  // ind variables to ensure vehicle chooses forward motion
  int neg_90_i_;
  int pos_90_i_;

  // helper functions
  void safety_bubble(std::vector<float> &ranges);
  std::vector<int> find_disparity_inds(const std::vector<float> ranges);
  void extend_disparities(std::vector<int> inds, std::vector<float> &ranges);
  void find_safe_region(int &gap_start_i, int &gap_end_i);
  void threshold_ranges(std::vector<float> &ranges);
  int find_best_dir(int start_i, int end_i, std::vector<float> ranges);
  void safe_cornering(const std::vector<float> ranges, float &steering_angle);
  float calc_speed(float steering_angle);
  
};

#endif  // GAPFOLLOW__GAPFOLLOW_HPP_

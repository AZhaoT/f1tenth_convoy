#ifndef LEADER_GF_HPP_
#define LEADER_GF_HPP_

#include <memory>
#include <iostream>
#include <limits>
#include <chrono>
#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "convoy_ros/msg/leader_gap_follow.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class LeaderGF : public rclcpp::Node
{
public:
    LeaderGF();

private:
    //subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_scan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;


    //publisher
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive;
    rclcpp::Publisher<convoy_ros::msg::LeaderGapFollow>::SharedPtr publisher_leader;

    
    void scan_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    // variables:
    const float angle1 = (-90) * M_PI / 180; // 90 deg to the right
    const float angle2 = (90) * M_PI / 180;  // 90 deg to the left
    const int bubble_size{80};
    int furthest_point_index{};
    float gap_dir{};
    float odom_x{};
    float odom_y{};
    float vel{};

    // helper functions:
    int find_furthest_point(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void get_gapdir(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void publish_drive();
    void publish_leader();

};

#endif // LEADER_GF_HPP_

#ifndef PUREPURSUIT__PUREPURSUIT_HPP_
#define PUREPURSUIT__PUREPURSUIT_HPP_

#include <memory>
#include <iostream>
#include <limits>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include "/sim_ws/src/convoy_ros/src/csv.h"
#include "math.h"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit();

private:
    // publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_rviz;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive;

    // subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;

    // subscriber callbacks
    void odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    // variables
    visualization_msgs::msg::Marker marker;
    std::vector<double> waypoint_x;
    std::vector<double> waypoint_y;
    std::vector<double> headings;
    double x_curr;
    double y_curr;
    int goal_index;
    double goal_dist;
    double look_ahead_dist_;
    double curvature;

    // helper functions
    double dist(double x0, double y0, double x1, double y1);
    void init_waypoint_marker(visualization_msgs::msg::Marker marker);
    int goal_index_finder();
    int closest_waypoint_finder();
    double get_yaw(const nav_msgs::msg::Odometry::SharedPtr msg);
    void get_curvature(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publish_marker(const visualization_msgs::msg::Marker marker);
    void publish_drive();
};

#endif // PUREPURSUIT__PUREPURSUIT_HPP_

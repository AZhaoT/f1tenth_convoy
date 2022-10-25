#ifndef FOLLOWER_PP_HPP_
#define FOLLOWER_PP_HPP_

#include <memory>
#include <iostream>
#include <limits>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include "math.h"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "convoy_ros/msg/leader_gap_follow.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class FollowerPP : public rclcpp::Node
{
public:
    FollowerPP();

private:
    // publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_rviz;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_follower;

    // subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Subscription<convoy_ros::msg::LeaderGapFollow>::SharedPtr subscription_leader;


    // subscriber callback
    void odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void leader_sub_callback(const convoy_ros::msg::LeaderGapFollow::SharedPtr leader_msg);


    // variables
    visualization_msgs::msg::Marker marker;
    geometry_msgs::msg::Point points;

    std::vector<double> waypoint_x;
    std::vector<double> waypoint_y;
    std::vector<double> headings;
    std::vector<double> waypoint_vel;
    double x_curr;
    double y_curr;
    int goal_index;
    double goal_dist;
    const double look_ahead_dist_ = 1.0;
    double curvature;
    double leader_dist;

    const unsigned int waypoints_num{2000};

    // helper functions
    double dist(double x0, double y0, double x1, double y1);
    void init_waypoint_marker(visualization_msgs::msg::Marker marker);
    void update_waypoint_marker(visualization_msgs::msg::Marker marker, const convoy_ros::msg::LeaderGapFollow::SharedPtr msg);
    int goal_index_finder();
    int closest_waypoint_finder();
    double get_yaw(const nav_msgs::msg::Odometry::SharedPtr msg);
    void get_curvature(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publish_marker(const visualization_msgs::msg::Marker marker);
    void publish_drive();
    void go_straight();
};

#endif // FOLLOWER_PP_HPP_

#include "convoy_ros/leader_gf.hpp"

LeaderGF::LeaderGF() : Node("leader_gf")
{
    // subscriber subscibes topic LaserScan
    subscription_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&LeaderGF::scan_sub_callback, this, _1));
    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "ego_racecar/odom", 10, std::bind(&LeaderGF::odom_sub_callback, this, _1));      

    // publisher publishes on topic scan_range
    publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    publisher_leader = this->create_publisher<convoy_ros::msg::LeaderGapFollow>("leader_gf", 10);
}

void LeaderGF::scan_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  furthest_point_index = find_furthest_point(scan_msg);
  std::cout << "test1";
  get_gapdir(scan_msg);
  publish_drive();
}

void LeaderGF::odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  odom_x = odom_msg->pose.pose.position.x;
  odom_y = odom_msg->pose.pose.position.y;
  publish_leader();
}

// find the furthest point inside the safety bubble using disparity
int LeaderGF::find_furthest_point(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  const int dist_angle1_index = (angle1 - scan_msg->angle_min) / scan_msg->angle_increment;
  const int dist_angle2_index = (angle2 - scan_msg->angle_min) / scan_msg->angle_increment;
  // scan the front 180 degress and find the largest gap
  // start from the right most scan
  int ind = dist_angle1_index;
  // find the largeset gap from -90 deg to 90 deg
  for (int i = dist_angle1_index; i < dist_angle2_index; i++)
  {
    // check for furthest point

    if ((scan_msg->ranges[i] - scan_msg->ranges[i - 1]) > 0.3) // disparity: obstacle to the right of the scan
    {
      for (int j = 0; j < bubble_size; j++)
        scan_msg->ranges[i + j] = scan_msg->ranges[i - 1]; // overwrite some elements after the disparity

      i += (bubble_size); // jump forward to the next unoverwritten element
    }
    else if ((scan_msg->ranges[i] - scan_msg->ranges[i + 1]) > 0.3) // disparity obstacle to the left of the scan
    {
      for (int j = 0; j < bubble_size; j++)
        scan_msg->ranges[i - j] = scan_msg->ranges[i + 1]; // overwrite some elements before the disparity

      i -= bubble_size; // jump backward to the earliest overwriten element
      ind -= bubble_size; // recall changes to the index due to disparity
    }
    else
    {
      // no disparity
      if (scan_msg->ranges[ind] < scan_msg->ranges[i])
      {
        ind = i;
      }
    }
  }

  return ind;
}

// derive gap direction in radian from furthest_point_index
void LeaderGF::get_gapdir(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  gap_dir = scan_msg->angle_min + furthest_point_index * scan_msg->angle_increment;
}

void LeaderGF::publish_drive()
{
  // publish drive msg
  auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msg.drive.steering_angle = gap_dir / 1.5;
  drive_msg.drive.steering_angle_velocity = 0.3;

  if (drive_msg.drive.steering_angle > 0.6)
    drive_msg.drive.speed = 0.5 / drive_msg.drive.steering_angle;
  else
    drive_msg.drive.speed = std::min(2.0, abs(1.0 / drive_msg.drive.steering_angle)); // max=8 for speilberg
  // 0.66 max steer

  // decelarate before corners by examining distance to furthest point

  vel = drive_msg.drive.speed;

  // publish
  publisher_drive->publish(drive_msg);
  // Publisher print:
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "steering angle  is:" << drive_msg.drive.steering_angle);
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "drive velocity  is:" << drive_msg.drive.speed);
}

void LeaderGF::publish_leader()
{
  auto leader_msg = convoy_ros::msg::LeaderGapFollow();
  leader_msg.leader_x = odom_x;
  leader_msg.leader_y = odom_y;
  leader_msg.leader_vel = vel;
  publisher_leader->publish(leader_msg);
}
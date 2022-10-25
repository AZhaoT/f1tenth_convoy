#include "convoy_ros/follower_pp.hpp"

FollowerPP::FollowerPP() : Node("follower_pp")
{
  // subscriber: location in map frame
  subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "opp_racecar/odom", 10, std::bind(&FollowerPP::odom_sub_callback, this, _1));
  subscription_leader= this->create_subscription<convoy_ros::msg::LeaderGapFollow>(
        "leader_gf", 10, std::bind(&FollowerPP::leader_sub_callback, this, _1));

  // publisher: waypoints and drive parameters
  publisher_rviz = this->create_publisher<visualization_msgs::msg::Marker>("rviz_m", 10);
  publisher_follower = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("opp_drive", 10);
  init_waypoint_marker(marker);
}

void FollowerPP::leader_sub_callback(const convoy_ros::msg::LeaderGapFollow::SharedPtr leader_msg)
{
  update_waypoint_marker(marker, leader_msg);
}

void FollowerPP::odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  // RCLCPP_INFO_STREAM(rclcpp:get_logger("rclcpp"), "Im here");

  // Get follower position
  x_curr = odom_msg->pose.pose.position.x;
  y_curr = odom_msg->pose.pose.position.y;

  if(static_cast<int>(waypoint_x.size()) < 500){
    go_straight();
    return;
  }
  // find the goal waypoint
  goal_index = goal_index_finder();
  goal_dist = dist(waypoint_x[goal_index], waypoint_y[goal_index], x_curr, y_curr);
  // std::cout << "my point: " << x_curr << ", " << y_curr << '\n';
  // std::cout << "goal point: " << waypoint_x[goal_index] << ", " << waypoint_y[goal_index] << '\n';
  // std::cout << "goal point index: " << goal_index << '\n'
  get_curvature(odom_msg);
  publish_marker(marker);
  publish_drive();
}

void FollowerPP::init_waypoint_marker(visualization_msgs::msg::Marker marker)
{
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
}

//helper functions

// compute distance between two points
double FollowerPP::dist(double x0, double y0, double x1, double y1)
{
  return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
}

//update leader waypoints and display in rviz
void FollowerPP::update_waypoint_marker(visualization_msgs::msg::Marker marker, const convoy_ros::msg::LeaderGapFollow::SharedPtr msg)
{
  //get leader position
  float x = msg->leader_x;
  float y = msg->leader_y;
  float v = msg->leader_vel;
  waypoint_x.push_back(x);
  waypoint_y.push_back(y);
  waypoint_vel.push_back(v);

  //calculate distance from leader
  leader_dist = dist(x, y, x_curr, y_curr);

  //update markers
  points.x = x;
  points.y = y;
  points.z = 0;
  marker.points.push_back(points);

  //remove waypoints that are no longer needed to maintain a fixed vector size
  while (waypoint_x.size() > waypoints_num)
  {
    waypoint_x.erase(waypoint_x.begin());
    waypoint_y.erase(waypoint_y.begin());
    waypoint_vel.erase(waypoint_vel.begin());
    marker.points.erase(marker.points.begin());
  }
}

//find the goal point's index in the vector "waypoint" when the robot is at the current position (x,y)
int FollowerPP::goal_index_finder()
{
  int curr_index = closest_waypoint_finder();
  bool found = false;
  int goal_index_temp = curr_index;
  double goal_dist_temp{};

  // find the closest goal point right outside the look ahead distance
  while (!found)
  {
    goal_dist_temp = dist(waypoint_x[goal_index_temp], waypoint_y[goal_index_temp], x_curr, y_curr);
    // if goal index reaches the last waypoint, then jump to the first waypoint
    if (goal_index_temp == static_cast<int>(waypoint_x.size()) - 1)
      goal_index_temp = 0;
    // go to the next point if the current goal point is still inside the look ahead distance
    if (goal_dist_temp < look_ahead_dist_)
      goal_index_temp++;
    else{
      found = true;
      goal_index = goal_index_temp;
      goal_dist = goal_dist_temp;
    }
  }
  return goal_index;
}

// find the closest waypoint to the current position (x,y) on the map
int FollowerPP::closest_waypoint_finder()
{
  double closest_dist{1.0};
  int curr_index{};
  for (int i = 0; i < static_cast<int>(waypoint_x.size()); i++)
  {
    if (dist(waypoint_x[i], waypoint_y[i], x_curr, y_curr) < closest_dist)
    {
      curr_index = i;
      closest_dist = dist(waypoint_x[i], waypoint_y[i], x_curr, y_curr);
    }
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "closest wp is:" << waypoint_x[curr_index]);

  return curr_index;
}

// compute the current yaw in the map frame
double FollowerPP::get_yaw(const nav_msgs::msg::Odometry::SharedPtr msg){
  return atan2(2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
                     -1.0 + 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.w + msg->pose.pose.orientation.x * msg->pose.pose.orientation.x));
}

// compute curvature to navigate to the goal point
void FollowerPP::get_curvature(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double yaw = get_yaw(msg);
  // std::cout << "my yaw: " << yaw << '\n';
  double heading_map = atan2(waypoint_y[goal_index] - y_curr, waypoint_x[goal_index] - x_curr);
  double look_ahead_angle = heading_map - yaw;
  // std::cout << "my tan is: " << atan2(waypoint_y[goal_index] - y_curr, waypoint_x[goal_index] - x_curr) / 3.14 * 180 << '\n';

  //correction for large positive heading_map and negative yaw, or vice versa
  if (look_ahead_angle > M_PI)
  {
    look_ahead_angle -= 2 * M_PI;
  }
  else if (look_ahead_angle < (-1.0) * M_PI)
  {
    look_ahead_angle += 2 * M_PI;
  }
  // std::cout << "my lookahead angle is: " << look_ahead_angle / 3.14 * 180 << '\n';

  // Find delta_y (delta_y is the displacement in the y-direction between vehical's current location and the waypoint in the robot's frame)
  // The x direction is the robot's moving direction
  double delta_y = goal_dist * sin(look_ahead_angle);

  // calculate turn curvature
  curvature = 2 * delta_y / (goal_dist * goal_dist);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "curvature is: " << curvature);
}

// publish rviz marker
void FollowerPP::publish_marker(const visualization_msgs::msg::Marker marker)
{
  auto rviz_msg = visualization_msgs::msg::Marker();
  rviz_msg = marker;
  publisher_rviz->publish(rviz_msg);
}

// publish drive speed and steering angle
void FollowerPP::publish_drive()
{
  auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  // set steering angle
  drive_msg.drive.steering_angle = curvature / 4;
  drive_msg.drive.steering_angle_velocity = 0.3;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "opp speed is:" << drive_msg.drive.speed);
  
  //set drive speed
  drive_msg.drive.speed = leader_dist / look_ahead_dist_ * waypoint_vel[goal_index];


  // if (drive_msg.drive.steering_angle > 0.6)
  //   drive_msg.drive.speed = abs(0.5 / drive_msg.drive.steering_angle);
  // else
  //   drive_msg.drive.speed = std::min(2.0, abs(1.0 / drive_msg.drive.steering_angle)); // max=8 for speilberg
  // 0.66 max steer
  // publish
  publisher_follower->publish(drive_msg);
}

// go straight if waypoint vector is not filled
void FollowerPP::go_straight()
{
  auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "going straight...");
  drive_msg.drive.speed = 2.0;
  publisher_follower->publish(drive_msg);
}

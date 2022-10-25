#include "convoy_ros/pure_pursuit.hpp"

PurePursuit::PurePursuit() : Node("pure_pursuit")
{
  //ROS parameters
  this->declare_parameter<double>("look_ahead_dist", 0.0);

  this->get_parameter("look_ahead_dist", look_ahead_dist_);

  // subscriber: location in map frame
  subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "ego_racecar/odom", 10, std::bind(&PurePursuit::odom_sub_callback, this, _1));

  // publisher: waypoints and drive parameters
  publisher_rviz = this->create_publisher<visualization_msgs::msg::Marker>("rviz_m", 10);
  publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
  init_waypoint_marker(marker);
}

void PurePursuit::odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  x_curr = odom_msg->pose.pose.position.x;
  y_curr = odom_msg->pose.pose.position.y;
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

void PurePursuit::init_waypoint_marker(visualization_msgs::msg::Marker marker)
{
  // read in all the data
  io::CSVReader<3> in("/sim_ws/src/convoy_ros/src/data.csv");
  double x;
  double y;
  double heading;
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
  geometry_msgs::msg::Point points;
  while (in.read_row(x, y, heading))
  {
    waypoint_x.push_back(x);
    waypoint_y.push_back(y);
    headings.push_back(heading);
    points.x = x;
    points.y = y;
    points.z = 0.0;
    marker.points.push_back(points);
    // markerArray.markers.push_back(marker);
  }
}

//helper functions

// compute distance between two points
double PurePursuit::dist(double x0, double y0, double x1, double y1)
{
  return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
}

//find the goal point's index in the vector "waypoint" when the robot is at the current position (x,y)
int PurePursuit::goal_index_finder()
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
int PurePursuit::closest_waypoint_finder()
{
  double closest_dist{100.0};
  int curr_index{};
  for (int i = 0; i < static_cast<int>(waypoint_x.size()); i++)
  {
    if (dist(waypoint_x[i], waypoint_y[i], x_curr, y_curr) < closest_dist)
    {
      curr_index = i;
      closest_dist = dist(waypoint_x[i], waypoint_y[i], x_curr, y_curr);
    }
  }
  return curr_index; 
}

// compute the current yaw in the map frame
double PurePursuit::get_yaw(const nav_msgs::msg::Odometry::SharedPtr msg){
  return atan2(2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
                     -1.0 + 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.w + msg->pose.pose.orientation.x * msg->pose.pose.orientation.x));
}

// compute curvature to navigate to the goal point
void PurePursuit::get_curvature(const nav_msgs::msg::Odometry::SharedPtr msg)
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
void PurePursuit::publish_marker(const visualization_msgs::msg::Marker marker)
{
  auto rviz_msg = visualization_msgs::msg::Marker();
  rviz_msg = marker;
  publisher_rviz->publish(rviz_msg);
}

// publish drive speed and steering angle
void PurePursuit::publish_drive()
{
  auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msg.drive.steering_angle = curvature / 2;
  drive_msg.drive.steering_angle_velocity = 0.3;

  if (drive_msg.drive.steering_angle > 0.6)
    drive_msg.drive.speed = abs(0.5 / drive_msg.drive.steering_angle);
  else
    drive_msg.drive.speed = std::min(8.0, abs(1.0 / drive_msg.drive.steering_angle)); // max=8 for speilberg
  // 0.66 max steer
  // publish
  publisher_drive->publish(drive_msg);
}

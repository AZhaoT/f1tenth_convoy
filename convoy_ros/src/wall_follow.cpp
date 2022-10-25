#include "convoy_ros/wall_follow.hpp"

WallFollow::WallFollow() : Node("wall_follow")
{
  // ROS parameters
  this->declare_parameter<float>("kp", 0.0);
  this->declare_parameter<float>("ki", 0.0);
  this->declare_parameter<float>("kd", 0.0);
  this->declare_parameter<float>("lookahead_dist", 0.0);
  this->declare_parameter<float>("dist_angle_const", 0.0);
  this->declare_parameter<float>("left_dist_des", 0.0);
  this->declare_parameter<float>("right_dist_des", 0.0);
  this->declare_parameter<float>("low_angle_speed", 0.0);
  this->declare_parameter<float>("low_angle", 0.0);
  this->declare_parameter<float>("med_angle_speed", 0.0);
  this->declare_parameter<float>("med_angle", 0.0);
  this->declare_parameter<float>("high_angle_speed", 0.0);
  this->declare_parameter<float>("max_angle", 0.0);
  this->declare_parameter<float>("theta", 0.0);

  this->declare_parameter<std::string>("drive_topic", "");
  this->declare_parameter<std::string>("scan_topic", "");

  this->get_parameter("kp", kp_);
  this->get_parameter("ki", ki_);
  this->get_parameter("kd", kd_);
  this->get_parameter("lookahead_dist", L_);
  this->get_parameter("dist_angle_const", dist_angle_const_);
  this->get_parameter("left_dist_des", left_dist_des_);
  this->get_parameter("right_dist_des", right_dist_des_);
  this->get_parameter("low_angle_speed", low_angle_speed_);
  this->get_parameter("low_angle", low_angle_);
  this->get_parameter("med_angle_speed", med_angle_speed_);
  this->get_parameter("med_angle", med_angle_);
  this->get_parameter("high_angle_speed", high_angle_speed_);
  this->get_parameter("max_angle", max_angle_);
  this->get_parameter("theta", theta_);

  this->get_parameter("drive_topic", drive_topic_); 
  this->get_parameter("scan_topic", scan_topic_);

  // convert angles to radians
  low_angle_ *= M_PI / 180.;
  med_angle_ *= M_PI / 180.;
  max_angle_ *= M_PI / 180.;
  theta_ *= M_PI / 180.;

  // publishers
  drive_pub_ = this->create_publisher<AckermannDriveStamped>(
      drive_topic_, 1
  );
  
  // subscribers
  scan_sub_ = this->create_subscription<LaserScan>(
      scan_topic_, 1, std::bind(&WallFollow::scan_cb, this, _1)
  );
  
  // variables
  prev_error_ = 0.0;
  integral_ = 0.0;
  zero_angle_= M_PI_2;
  angle_range_ = 0.0;

  theta_ = zero_angle_ - theta_;
}

// subscriber callbacks

void WallFollow::scan_cb(const LaserScan::SharedPtr msg)
{
  pid_control(msg);
}

// helper functions

float WallFollow::calc_error(float a, float b)
{
  float alpha = atan2(a * std::cos(theta_) - b, a * std::sin(theta_));
  float D_t = b * std::cos(alpha);
  float D_t_plus_1 = D_t + L_ * std::sin(alpha);
  return -dist_angle_const_ * (left_dist_des_ - D_t_plus_1);
}

float WallFollow::calc_speed(float steering_angle)
{
  if (std::abs(steering_angle) < low_angle_)
    return low_angle_speed_;
  else if (std::abs(steering_angle) < med_angle_)
    return med_angle_speed_;
  else if (std::abs(steering_angle) < max_angle_)
    return high_angle_speed_;
  else
    return 0.0;
}

float WallFollow::calc_steer_angle(float error)
{
  // PID controller used to calculate steering angle
  integral_ += error;
  float derivative = (prev_error_ + error) / 2.0;
  return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

long WallFollow::get_angle_ind(const LaserScan::SharedPtr msg, float angle)
{
  return std::round((angle - msg->angle_min) / msg->angle_increment) - 1;
}

float WallFollow::get_range(const LaserScan::SharedPtr msg, float angle)
{
  long angle_ind = get_angle_ind(msg, angle);
  return msg->ranges[angle_ind];
}

void WallFollow::pid_control(const LaserScan::SharedPtr msg)
{
  // use PID controller to calculate steering angle
  float b = get_range(msg, zero_angle_);
  float a = get_range(msg, theta_);
  float error = calc_error(a, b);
  float steering_angle = calc_steer_angle(error);
  float speed = calc_speed(steering_angle);

  AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = this->now();
  drive_msg.drive.speed = speed;
  drive_msg.drive.steering_angle = steering_angle;
  drive_pub_->publish(drive_msg);
}

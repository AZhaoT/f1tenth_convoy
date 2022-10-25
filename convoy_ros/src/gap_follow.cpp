#include "convoy_ros/gap_follow.hpp"

GapFollow::GapFollow() : Node("gap_follow")
{
  // ROS parameters
  this->declare_parameter<float>("car_radius", 0.0);
  this->declare_parameter<float>("scan_threshold_m", 0.0);
  this->declare_parameter<float>("disparity_threshold_m", 0.0);
  this->declare_parameter<float>("cornering_threshold_m", 0.0);
  this->declare_parameter<float>("low_angle_speed", 0.0);
  this->declare_parameter<float>("low_angle", 0.0);
  this->declare_parameter<float>("med_angle_speed", 0.0);
  this->declare_parameter<float>("med_angle", 0.0);
  this->declare_parameter<float>("high_angle_speed", 0.0);
  this->declare_parameter<float>("max_angle", 0.0);

  this->declare_parameter<std::string>("drive_topic", "");
  this->declare_parameter<std::string>("scan_topic", "");

  this->get_parameter("car_radius", car_radius_);
  this->get_parameter("scan_threshold_m", scan_thresh_m_);
  this->get_parameter("disparity_threshold_m", disparity_thresh_m_);
  this->get_parameter("cornering_threshold_m", cornering_thresh_m_);
  this->get_parameter("low_angle_speed", low_angle_speed_);
  this->get_parameter("low_angle", low_angle_);
  this->get_parameter("med_angle_speed", med_angle_speed_);
  this->get_parameter("med_angle", med_angle_);
  this->get_parameter("high_angle_speed", high_angle_speed_);
  this->get_parameter("max_angle", max_angle_);

  this->get_parameter("drive_topic", drive_topic_);
  this->get_parameter("scan_topic", scan_topic_);

  low_angle_ *= M_PI / 180.;
  med_angle_ *= M_PI / 180.;
  max_angle_ *= M_PI / 180.;

  // publishers
  drive_pub_ = this->create_publisher<AckermannDriveStamped>(
      drive_topic_, 1
  );
  proc_scan_pub_ = this->create_publisher<LaserScan>(
      "proc_scan", 1
  );

  // subscribers
  scan_sub_ = this->create_subscription<LaserScan>(
      scan_topic_, 1, std::bind(&GapFollow::scan_cb, this, _1)
  );

  // variables
  angle_inc_ = 0.0;
}

// subscriber callbacks

void GapFollow::scan_cb(const LaserScan::SharedPtr msg)
{
  if (angle_inc_ == 0.0) {
    // set up some variables
    angle_inc_ = msg->angle_increment;
    num_scans_ = std::size(msg->ranges);
    neg_90_i_ = std::round((-M_PI_2 - msg->angle_min) / angle_inc_);
    pos_90_i_ = std::round((M_PI_2 - msg->angle_min) / angle_inc_);
  }
  std::vector<float> proc_ranges = msg->ranges;

  safety_bubble(proc_ranges);

  std::vector<int> disp_inds = find_disparity_inds(proc_ranges);
  if (std::size(disp_inds) > 0) {
    extend_disparities(disp_inds, proc_ranges);
  }
  
  int gap_start_i, gap_end_i;
  find_safe_region(gap_start_i, gap_end_i);

  threshold_ranges(proc_ranges);

  int best_i = find_best_dir(gap_start_i, gap_end_i, proc_ranges);
  float steering_angle = best_i * angle_inc_ + msg->angle_min;

  safe_cornering(proc_ranges, steering_angle);
  RCLCPP_INFO(this->get_logger(), "steering angle: %f", steering_angle);

  // calculate safe speed based on steering angle
  float speed = calc_speed(steering_angle);
  RCLCPP_INFO(this->get_logger(), "speed: %f", speed);

  // publish processed scan for visualization
  LaserScan proc_scan_msg = *msg;
  proc_scan_msg.ranges = proc_ranges;
  proc_scan_pub_->publish(proc_scan_msg);

  // publish drive message
  AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = this->now();
  drive_msg.drive.speed = speed;
  drive_msg.drive.steering_angle = steering_angle;
  drive_pub_->publish(drive_msg);
}

// helper functions

void GapFollow::safety_bubble(std::vector<float> &ranges)
{
  // find closest point to LIDAR and place bubble of zeros around it
  auto min_it = std::min_element(ranges.begin(), ranges.end());
  int min_ind = std::distance(ranges.begin(), min_it);
  float min_range = *min_it;
  // eliminate all points in car radius bubble (set to zero)
  float bubble_ang_2 = atan2(car_radius_, min_range);
  int bubble_width_2 = bubble_ang_2 / angle_inc_ + 1;
  safety_start_i_ = std::max(0, min_ind - bubble_width_2);
  safety_end_i_ = std::min(num_scans_ - 1, min_ind + bubble_width_2);
  for (int i = safety_start_i_; i <= safety_end_i_; ++i)
    ranges[i] = 0.0;
}

std::vector<int> GapFollow::find_disparity_inds(const std::vector<float> ranges)
{
  std::vector<int> disp_inds;
  for (int i = 0; i < num_scans_ - 1; ++i) {
    if (std::abs(ranges[i+1] - ranges[i]) > disparity_thresh_m_) {
      disp_inds.push_back(i);
    }
  }
  return disp_inds;
}

void GapFollow::extend_disparities(
    std::vector<int> inds, std::vector<float> &ranges
) {
  for (const int& i : inds) {
    if ((ranges[i+1] - ranges[i]) > disparity_thresh_m_) {
      float extend_ang = atan2(car_radius_, ranges[i]);
      int extend_inds = extend_ang / angle_inc_ + 1;
      for (int j = i + 1; j < std::min(num_scans_, i + extend_inds); ++j)
        ranges[j] = ranges[i];
    }
    else if ((ranges[i] - ranges[i+1]) > disparity_thresh_m_) {
      float extend_ang = atan2(car_radius_, ranges[i+1]);
      int extend_inds = extend_ang / angle_inc_ + 1;
      for (int j = i; j >= std::max(0, i - extend_inds); --j)
        ranges[j] = ranges[i+1];
    }
  }
}

void GapFollow::find_safe_region(int &gap_start_i, int &gap_end_i)
{
  // update gap_start_i and gap_end_i to be inds indicating direction to go 
  // further from zero bubble (where nearest object is)
  if (safety_end_i_ < neg_90_i_) {
    // zero area is behind to the right of the vehicle
    gap_start_i = neg_90_i_;
    gap_end_i = pos_90_i_;
  }
  else if (safety_start_i_ > pos_90_i_) {
    // zero area is behind to the left of the vehicle
    gap_start_i = neg_90_i_;
    gap_end_i = pos_90_i_;
  }
  else if ((safety_start_i_ - neg_90_i_) > (pos_90_i_ - safety_end_i_)) {
    // zero area is closer to the right of the vehicle, i.e., go left
    gap_start_i = neg_90_i_;
    gap_end_i = safety_start_i_ - 1;
  }
  else {
    // zero area is closer to the left of the vehicle, i.e., go right 
    gap_start_i = safety_end_i_ + 1;
    gap_end_i = pos_90_i_;
  }
}

void GapFollow::threshold_ranges(std::vector<float> &ranges)
{
  for (auto &r : ranges) {
    if (r > scan_thresh_m_) {
      r = scan_thresh_m_;
    }
  }
}

int GapFollow::find_best_dir(
    int start_i, int end_i, std::vector<float> ranges
) {
  // start_i and end_i are start and end indices of max-gap range
  // returns: index of best point in ranges b/w start_i and end_i
  auto start_it = ranges.begin() + start_i;
  auto end_it = ranges.begin() + end_i;
  auto max_it = std::max_element(start_it, end_it);
  if (*max_it == scan_thresh_m_) {
    int current_gap_start, current_gap_end;
    int current_gap_width = 0;
    int deepest_gap_start, deepest_gap_end;
    int deepest_gap_width = 0;
    bool on_max = false;
    for (int i = start_i; i < end_i; ++i) {
      if (ranges[i] == scan_thresh_m_) {
        if (on_max) {
          current_gap_end = i;
          current_gap_width = current_gap_end - current_gap_start;
        }
        else {  // if !(on_max)
          on_max = true;
          current_gap_start = i;
        }
      }
      else {  // if (ranges[i] != scan_thresh_m_)
        if (on_max) {
          on_max = false;
          if (current_gap_width > deepest_gap_width) {
            deepest_gap_start = current_gap_start;
            deepest_gap_end = current_gap_end;
            deepest_gap_width = current_gap_width;
          }
        }
      }
    }
    int best_i = (deepest_gap_start + deepest_gap_end) / 2;
    return best_i;
  }
  else {
    int best_i = std::distance(ranges.begin(), max_it); // max dist
    return best_i;
  }
}

void GapFollow::safe_cornering(
    const std::vector<float> ranges, float &steering_angle
) {
  // if any point on the side of the car in the direction the car is turning is 
  // below a safe distance, set steering angle to zero and go straight
  if (steering_angle > 0.0) {
    float min_range = *std::min_element(
        ranges.begin(), ranges.begin() + neg_90_i_
    );
    if (min_range < cornering_thresh_m_)
      steering_angle = 0.0;
  }
  else {  // if (steering_angle <= 0.0)
    float min_range = *std::min_element(
        ranges.begin() + pos_90_i_, ranges.end()
    );
    if (min_range < cornering_thresh_m_)
      steering_angle = 0.0;
  }
}

float GapFollow::calc_speed(float steering_angle)
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

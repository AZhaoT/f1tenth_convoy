#include "convoy_ros/follower_pp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowerPP>());
  rclcpp::shutdown();
  return 0;
}

#include "convoy_ros/leader_gf.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeaderGF>());
  rclcpp::shutdown();
  return 0;
}
#include "convoy_ros/wall_follow.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollow>());
  rclcpp::shutdown();
  return 0;
}

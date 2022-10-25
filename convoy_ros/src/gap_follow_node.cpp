#include "convoy_ros/gap_follow.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GapFollow>());
  rclcpp::shutdown();
  return 0;
}

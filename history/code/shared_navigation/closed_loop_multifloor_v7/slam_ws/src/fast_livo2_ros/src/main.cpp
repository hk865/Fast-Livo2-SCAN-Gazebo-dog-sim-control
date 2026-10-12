#include <runner.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("laserMapping");
  fast_livo2_core::run_mapping(nh);
  rclcpp::shutdown();
  return 0;
}

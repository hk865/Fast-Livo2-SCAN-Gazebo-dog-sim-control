#include <runner.hpp>
#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include <iostream>
namespace {void normal_stop(int){fast_livo2_core::request_mapping_stop();}}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGUSR1,normal_stop);
  try {
  auto nh = std::make_shared<rclcpp::Node>("laserMapping");
  fast_livo2_core::run_mapping(nh);
  rclcpp::shutdown();
  return 0;
  }catch(const std::exception& e){std::cerr<<"V19 mapping failure: "<<e.what()<<std::endl;rclcpp::shutdown();return 2;}
}

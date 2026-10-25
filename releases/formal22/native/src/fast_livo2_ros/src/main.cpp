#include <runner.hpp>
#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include <cstdio>
#include <iostream>
namespace {
void normal_stop(int){fast_livo2_core::request_mapping_stop();}
int mapping_failure_exit(const char* first_error)
{
  std::fprintf(stderr, "V19 mapping failure: %.512s\n", first_error);
  try { rclcpp::shutdown(); }
  catch (const std::exception& secondary) {
    std::fprintf(stderr, "V19 secondary ROS shutdown failure: %.512s\n", secondary.what());
  } catch (...) {
    std::fprintf(stderr, "V19 secondary ROS shutdown failure: non-std exception\n");
  }
  return 2;
}
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGUSR1,normal_stop);
  try {
  auto nh = std::make_shared<rclcpp::Node>("laserMapping");
  fast_livo2_core::run_mapping(nh);
  rclcpp::shutdown();
  return 0;
  }catch(const std::exception& e){return mapping_failure_exit(e.what());}
  catch(...){return mapping_failure_exit("non-std exception");}
}

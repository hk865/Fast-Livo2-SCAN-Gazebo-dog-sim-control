#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joint_state.h>
#include <chrono>
#include <cstdio>
#include <thread>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("publish_boundary_fixture");
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS().keep_last(2000));
  auto decoy = node->create_publisher<sensor_msgs::msg::JointState>("/other_joint_states", rclcpp::SensorDataQoS().keep_last(2000));
  auto until = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < until) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (!pub->get_subscription_count()) { rclcpp::shutdown(); return 2; }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::vector<int64_t> elapsed_ns;
  for (int i = 0; i < 500; ++i) {
    sensor_msgs::msg::JointState m;
    const int64_t ns = 1378123456789LL + int64_t(i) * 1000000LL;
    m.header.stamp.sec = ns / 1000000000LL;
    m.header.stamp.nanosec = ns % 1000000000LL;
    m.header.frame_id = "actual_jstate_fixture";
    m.name = {"lf_hip_joint", "rf_hip_joint"};
    m.position = {double(i) / 10., -double(i) / 10.};
    m.velocity = {double(i) / 20., -double(i) / 20.};
    m.effort = {double(i) / 30., -double(i) / 30.};
    auto enter = std::chrono::steady_clock::now();
    pub->publish(m);
    elapsed_ns.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now()-enter).count());
    decoy->publish(m);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // Valid C JointState on the same topic has a different in-memory ABI.
  // The observer must tag it unsupported rather than reinterpret its fields.
  auto cpub = rcl_get_zero_initialized_publisher();
  auto options = rcl_publisher_get_default_options();
  options.qos = rmw_qos_profile_sensor_data;
  auto ts = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);
  int ci = rcl_publisher_init(&cpub, node->get_node_base_interface()->get_rcl_node_handle(), ts, "/joint_states", &options);
  sensor_msgs__msg__JointState cmessage;
  sensor_msgs__msg__JointState__init(&cmessage);
  cmessage.header.stamp.sec = 1378; cmessage.header.stamp.nanosec = 999999999;
  int cr = ci == RCL_RET_OK ? rcl_publish(&cpub, &cmessage, nullptr) : -1;
  int cf = ci == RCL_RET_OK ? rcl_publisher_fini(&cpub, node->get_node_base_interface()->get_rcl_node_handle()) : -1;
  sensor_msgs__msg__JointState__fini(&cmessage);
  std::printf("{\"cpp_publish_elapsed_ns\":[");
  for (size_t i=0;i<elapsed_ns.size();++i) std::printf("%s%lld",i?",":"",(long long)elapsed_ns[i]);
  std::printf("],\"C_type_init_ret\":%d,\"C_type_publish_ret\":%d,\"C_type_fini_ret\":%d}\n",ci,cr,cf);
  rclcpp::shutdown();
  return 0;
}

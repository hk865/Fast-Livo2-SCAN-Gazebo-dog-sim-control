cmake_minimum_required(VERSION 3.16)
project(champ_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Imu.msg"
  "msg/PID.msg"
  "msg/Velocities.msg"
  "msg/Pose.msg"
  "msg/Joints.msg"
  "msg/Contacts.msg"
  "msg/PointArray.msg"
  "msg/Point.msg"
  "msg/ContactsStamped.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(unitree_go2_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY rviz 
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/imu_filter_node.py
  scripts/reset_manager.py
  scripts/publish_initial_joints.py
  scripts/ground_truth_tf.py
  scripts/tf_chain_diagnose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

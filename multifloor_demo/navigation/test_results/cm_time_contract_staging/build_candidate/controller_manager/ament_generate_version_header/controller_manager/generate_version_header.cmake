# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Generated from generate_version_header.cmake.in
# This file is used by ament_generate_version_header()

set(GENERATED_HEADER_FILE "/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/navigation/test_results/cm_time_contract_staging/build_candidate/controller_manager/ament_generate_version_header/controller_manager/controller_manager/version.h")
set(VERSION_TEMPLATE_FILE "/opt/ros/jazzy/share/ament_cmake_gen_version_h/cmake/version.h.in")

set(VERSION_MAJOR "4")
set(VERSION_MINOR "45")
set(VERSION_PATCH "2")
set(VERSION_STR "4.45.2")

set(PROJECT_NAME_UPPER "CONTROLLER_MANAGER")

configure_file("${VERSION_TEMPLATE_FILE}" "${GENERATED_HEADER_FILE}")

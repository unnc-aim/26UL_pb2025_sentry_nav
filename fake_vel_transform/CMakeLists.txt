cmake_minimum_required(VERSION 3.10)
project(fake_vel_transform)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/fake_vel_transform.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fake_vel_transform::FakeVelTransform
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_imu_heading
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_imu_heading.py
      $<TARGET_FILE:fake_vel_transform_node>)
  set_tests_properties(test_imu_heading PROPERTIES TIMEOUT 30
    ENVIRONMENT "ROS_DOMAIN_ID=94;ROS_LOCALHOST_ONLY=1")
  add_test(NAME test_remote_spin
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_remote_spin.py
      $<TARGET_FILE:fake_vel_transform_node>)
  set_tests_properties(test_remote_spin PROPERTIES TIMEOUT 40
    ENVIRONMENT "ROS_DOMAIN_ID=96;ROS_LOCALHOST_ONLY=1")
  set(ament_cmake_clang_format_CLANG_FORMAT_VERSION 21)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)

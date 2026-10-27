cmake_minimum_required(VERSION 3.10)
project(loam_interface)

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
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN loam_interface::LoamInterfaceNode
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_odometry_callbacks test/test_odometry_callbacks.cpp)
  target_link_libraries(test_odometry_callbacks ${PROJECT_NAME})
  ament_target_dependencies(test_odometry_callbacks rclcpp nav_msgs sensor_msgs tf2_ros)
  add_test(NAME odometry_during_cloud_work COMMAND test_odometry_callbacks)
  set_tests_properties(odometry_during_cloud_work PROPERTIES
    TIMEOUT 15 ENVIRONMENT "ROS_DOMAIN_ID=120;ROS_LOCALHOST_ONLY=1")
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

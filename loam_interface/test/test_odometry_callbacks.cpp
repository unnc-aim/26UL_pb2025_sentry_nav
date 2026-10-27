#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "loam_interface/loam_interface.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"state_estimation_topic", "/callback_test/odom"},
    {"registered_scan_topic", "/callback_test/cloud"},
    {"lidar_frame", "test_imu"}, {"base_frame", "test_base"}});
  auto bridge = std::make_shared<loam_interface::LoamInterfaceNode>(options);
  auto input = std::make_shared<rclcpp::Node>("callback_test_input");
  tf2_ros::StaticTransformBroadcaster transforms(input);
  geometry_msgs::msg::TransformStamped mount;
  mount.header.stamp = input->now();
  mount.header.frame_id = "test_base";
  mount.child_frame_id = "test_imu";
  mount.transform.translation.z = .43;
  mount.transform.rotation.w = 1;
  transforms.sendTransform(mount);
  auto publisher = input->create_publisher<nav_msgs::msg::Odometry>(
    "/callback_test/odom", 10);
  std::atomic<int> received{0}, during_work{0};
  std::atomic<bool> busy{false}, entered{false}, geometry_ok{true};
  auto subscription = input->create_subscription<nav_msgs::msg::Odometry>(
    "/lidar_odometry", 10, [&](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      ++received;
      if (busy) {++during_work;}
      if (std::abs(msg->pose.pose.position.x - 2.) > 1e-6 ||
        std::abs(msg->pose.pose.position.z - .43) > 1e-6)
      {
        geometry_ok = false;
      }
    });
  rclcpp::CallbackGroup::SharedPtr cloud_group;
  bridge->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
    if (group->find_subscription_ptrs_if([](rclcpp::SubscriptionBase::SharedPtr sub) {
      return std::string(sub->get_topic_name()) == "/callback_test/cloud";
    })) {cloud_group = group;}
  });
  if (!cloud_group) {
    std::cerr << "Cloud callback group missing\n";
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(bridge);
  executor.add_node(input);
  std::thread spin([&]() {executor.spin();});
  auto publish = [&]() {
      nav_msgs::msg::Odometry msg;
      msg.header.stamp = input->now();
      msg.pose.pose.position.x = 2.;
      msg.pose.pose.orientation.w = 1.;
      publisher->publish(msg);
    };
  const auto end = std::chrono::steady_clock::now() + 4s;
  while (received < 3 && std::chrono::steady_clock::now() < end) {
    publish();
    std::this_thread::sleep_for(20ms);
  }
  // Simulate a slow transform/publication in the actual cloud callback group.
  auto work = bridge->create_wall_timer(10ms, [&]() {
    if (entered.exchange(true)) {return;}
    busy = true;
    std::this_thread::sleep_for(500ms);
    busy = false;
  }, cloud_group);
  const auto until = std::chrono::steady_clock::now() + 800ms;
  while (std::chrono::steady_clock::now() < until) {
    publish();
    std::this_thread::sleep_for(20ms);
  }
  executor.cancel();
  spin.join();
  rclcpp::shutdown();
  const bool passed = entered && during_work >= 3 && geometry_ok;
  std::cout << "Odometry received during cloud work: " << during_work
            << "; transform correct: " << geometry_ok << '\n';
  return passed ? 0 : 1;
}

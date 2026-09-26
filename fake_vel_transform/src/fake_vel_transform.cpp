// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fake_vel_transform/fake_vel_transform.hpp"

#include <cmath>
#include <stdexcept>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<float>("init_spin_speed", 0.0);
  this->declare_parameter<double>("translation_scale", 1.0);
  max_odom_age_ = this->declare_parameter<double>("max_odom_age", 0.5);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("init_spin_speed", spin_speed_);
  this->get_parameter("translation_scale", translation_scale_);
  if (
    !std::isfinite(max_odom_age_) || max_odom_age_ <= 0.0 || !std::isfinite(translation_scale_) ||
    !std::isfinite(spin_speed_)) {
    throw std::invalid_argument(
      "Velocity parameters must be finite; max_odom_age must be positive");
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>(output_cmd_vel_topic_, 1);

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  spin_speed_ = std::isfinite(msg->data) ? msg->data : 0.0f;
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  const auto & q = msg->pose.pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  const auto stamp = rclcpp::Time(msg->header.stamp);
  const double age = (now() - stamp).seconds();
  if (
    msg->child_frame_id != robot_base_frame_ || !std::isfinite(norm) ||
    std::abs(norm - 1.0) > 0.01 || age < -0.05 || age > max_odom_age_) {
    odom_ready_ = false;
    return;
  }
  if (odom_ready_ && stamp < odom_stamp_) {
    return;
  }
  current_robot_base_angle_ = tf2::getYaw(q);
  odom_stamp_ = stamp;
  odom_ready_ = true;
  publishTransform();
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double age = (now() - odom_stamp_).seconds();
  if (
    odom_ready_ && age >= -0.05 && age <= max_odom_age_ && std::isfinite(msg->linear.x) &&
    std::isfinite(msg->linear.y) && std::isfinite(msg->angular.z)) {
    // The fake frame follows the odometry axes; rotate each new command into the gimbal frame.
    cmd_vel_chassis_pub_->publish(transformVelocity(msg, current_robot_base_angle_));
    return;
  }
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp = now();
  stop.header.frame_id = robot_base_frame_;
  cmd_vel_chassis_pub_->publish(stop);
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000, "Waiting for fresh odometry and finite velocity input");
}

void FakeVelTransform::publishTransform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = odom_stamp_;
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

geometry_msgs::msg::TwistStamped FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff)
{
  geometry_msgs::msg::TwistStamped aft_tf_vel;
  aft_tf_vel.header.stamp = this->get_clock()->now();
  aft_tf_vel.header.frame_id = robot_base_frame_;
  aft_tf_vel.twist.angular.z = twist->angular.z + spin_speed_;
  // Adapt the hardware velocity interface once; simulation keeps the standard sign.
  aft_tf_vel.twist.linear.x =
    translation_scale_ * (twist->linear.x * cos(yaw_diff) + twist->linear.y * sin(yaw_diff));
  aft_tf_vel.twist.linear.y =
    translation_scale_ * (-twist->linear.x * sin(yaw_diff) + twist->linear.y * cos(yaw_diff));
  return aft_tf_vel;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)

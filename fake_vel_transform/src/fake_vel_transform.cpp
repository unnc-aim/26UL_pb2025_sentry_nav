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

#include <algorithm>
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
  gimbal_imu_topic_ = this->declare_parameter<std::string>("gimbal_imu_topic", "");
  max_imu_age_ = this->declare_parameter<double>("max_imu_age", 0.05);
  const auto remote_spin_topic = this->declare_parameter<std::string>("remote_spin_topic", "");

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
    !std::isfinite(spin_speed_) || !std::isfinite(max_imu_age_) || max_imu_age_ <= 0.0) {
    throw std::invalid_argument(
      "Velocity parameters must be finite; input age limits must be positive");
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>(output_cmd_vel_topic_, 1);

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  if (!remote_spin_topic.empty()) {
    remote_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
      remote_spin_topic, 1, [this](example_interfaces::msg::Float32::ConstSharedPtr msg) {
        remote_spin_speed_ = std::isfinite(msg->data) ? msg->data : 0.0f;
        remote_spin_stamp_ = now();
      });
  }
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  if (!gimbal_imu_topic_.empty()) {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      gimbal_imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FakeVelTransform::imuCallback, this, std::placeholders::_1));
  }
}

void FakeVelTransform::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
{
  const auto & q = msg->orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
  if (stamp <= 0 || !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01 ||
    msg->orientation_covariance[0] < 0.0)
  {
    imu_headings_.clear();
    return;
  }
  if (!imu_headings_.empty() && stamp <= imu_headings_.back().stamp) {
    return;
  }
  imu_headings_.push_back({stamp, tf2::getYaw(q)});
  const double history = std::max(2.0, max_odom_age_ + max_imu_age_);
  while (imu_headings_.size() > 2 &&
    (static_cast<double>(stamp - imu_headings_.front().stamp) * 1e-9 > history ||
    imu_headings_.size() > 10000))
  {
    imu_headings_.pop_front();
  }
}

bool FakeVelTransform::currentHeading(double & heading, rclcpp::Time & stamp) const
{
  heading = current_robot_base_angle_;
  stamp = now();
  if (gimbal_imu_topic_.empty()) {
    return true;
  }
  if (imu_headings_.empty()) {
    return false;
  }
  const auto & latest = imu_headings_.back();
  const double age = static_cast<double>(stamp.nanoseconds() - latest.stamp) * 1e-9;
  const int64_t reference = odom_stamp_.nanoseconds();
  if (age < -0.05 || age > max_imu_age_ || reference < imu_headings_.front().stamp ||
    reference > latest.stamp)
  {
    return false;
  }
  const auto upper = std::lower_bound(
    imu_headings_.begin(), imu_headings_.end(), reference,
    [](const ImuHeading & sample, int64_t time) {return sample.stamp < time;});
  double previous_yaw = upper->yaw;
  if (upper->stamp != reference) {
    const auto lower = std::prev(upper);
    const int64_t gap = upper->stamp - lower->stamp;
    if (static_cast<double>(gap) * 1e-9 > max_imu_age_) {
      return false;
    }
    const double delta = std::atan2(
      std::sin(upper->yaw - lower->yaw), std::cos(upper->yaw - lower->yaw));
    previous_yaw = lower->yaw + delta * static_cast<double>(reference - lower->stamp) / gap;
  }
  // IMU world origins cancel in the change since the odometry measurement.
  const double delta = std::atan2(
    std::sin(latest.yaw - previous_yaw), std::cos(latest.yaw - previous_yaw));
  heading = std::atan2(std::sin(heading + delta), std::cos(heading + delta));
  stamp = rclcpp::Time(latest.stamp, get_clock()->get_clock_type());
  return true;
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
  double heading = 0.0;
  rclcpp::Time heading_stamp(0, 0, get_clock()->get_clock_type());
  if (
    odom_ready_ && age >= -0.05 && age <= max_odom_age_ && std::isfinite(msg->linear.x) &&
    std::isfinite(msg->linear.y) && std::isfinite(msg->angular.z) &&
    currentHeading(heading, heading_stamp)) {
    // The fake frame follows the odometry axes; rotate each new command into the gimbal frame.
    cmd_vel_chassis_pub_->publish(transformVelocity(msg, heading, heading_stamp));
    return;
  }
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp = now();
  stop.header.frame_id = robot_base_frame_;
  cmd_vel_chassis_pub_->publish(stop);
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000, "Waiting for fresh heading, odometry and finite velocity input");
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
  const geometry_msgs::msg::Twist::SharedPtr & twist, double yaw_diff,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TwistStamped aft_tf_vel;
  aft_tf_vel.header.stamp = stamp;
  aft_tf_vel.header.frame_id = robot_base_frame_;
  double spin = spin_speed_;
  if (remote_spin_sub_ && cmd_spin_sub_->get_publisher_count() == 0) {
    // Explicit behavior-tree spin, including zero, takes precedence while its publisher exists.
    // Clear the old command when that publisher exits, before the next tree starts.
    spin_speed_ = 0.0f;
    const double age = (now() - remote_spin_stamp_).seconds();
    spin = remote_spin_stamp_.nanoseconds() > 0 && age >= 0.0 && age < 0.2 ?
      remote_spin_speed_ : 0.0;
  }
  aft_tf_vel.twist.angular.z = twist->angular.z + spin;
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

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

#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include <cmath>
#include <stdexcept>

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sensor_scan_generation
{

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options)
{
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");
  const double sync_min_interval = this->declare_parameter<double>("sync_min_interval", 0.0);
  const double sync_max_interval = this->declare_parameter<double>("sync_max_interval", 0.05);
  tf_timeout_ = this->declare_parameter<double>("tf_timeout", 0.05);
  max_input_age_ = this->declare_parameter<double>("max_input_age", 0.5);
  if (!std::isfinite(sync_min_interval) || sync_min_interval < 0.0) {
    throw std::invalid_argument("sync_min_interval must be finite and nonnegative");
  }
  if (
    !std::isfinite(sync_max_interval) || sync_max_interval <= 0.0 ||
    !std::isfinite(tf_timeout_) || tf_timeout_ < 0.0 ||
    !std::isfinite(max_input_age_) || max_input_age_ <= 0.0) {
    throw std::invalid_argument(
      "sync_max_interval and max_input_age must be positive; tf_timeout must be nonnegative");
  }

  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);

  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  odometry_sub_.subscribe(this, "lidar_odometry", qos_profile);
  // Publish chassis pose as soon as lidar odometry arrives, independently of clouds.
  odometry_sub_.registerCallback(
    std::bind(&SensorScanGenerationNode::odometryHandler, this, std::placeholders::_1));
  // loam_interface publishes reliable clouds; retain retransmission for their fragments.
  auto cloud_qos = qos_profile;
  cloud_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  cloud_qos.depth = 5;
  laser_cloud_sub_.subscribe(this, "registered_scan", cloud_qos);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(20), odometry_sub_, laser_cloud_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_max_interval));
  sync_->setInterMessageLowerBound(0, rclcpp::Duration::from_seconds(sync_min_interval));
  sync_->setInterMessageLowerBound(1, rclcpp::Duration::from_seconds(sync_min_interval));
  sync_->registerCallback(std::bind(
    &SensorScanGenerationNode::laserCloudAndOdometryHandler, this, std::placeholders::_1,
    std::placeholders::_2));
}

void SensorScanGenerationNode::odometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg)
{
  const rclcpp::Time stamp(odometry_msg->header.stamp);
  if (
    !isFresh(stamp) ||
    (previous_odometry_valid_ && stamp <= previous_odometry_stamp_)) {
    return;
  }

  tf2::Transform tf_lidar_to_chassis;
  tf2::Transform tf_lidar_to_robot_base;
  tf2::Transform tf_odom_to_chassis;
  tf2::Transform tf_odom_to_robot_base;
  tf2::Transform tf_odom_to_lidar;

  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);
  try {
    tf_lidar_to_robot_base = getTransform(lidar_frame_, robot_base_frame_, stamp);
    tf_lidar_to_chassis = getTransform(lidar_frame_, base_frame_, stamp);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "TF lookup failed at odometry time: %s. Skipping odometry.", ex.what());
    return;
  }
  if (!isFresh(stamp)) {
    return;
  }

  tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis;
  tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_to_robot_base;

  publishTransform(
    tf_odom_to_chassis, odometry_msg->header.frame_id, base_frame_, stamp);
  publishOdometry(
    tf_odom_to_robot_base, odometry_msg->header.frame_id, robot_base_frame_, stamp);
}

void SensorScanGenerationNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  if (
    !isFresh(rclcpp::Time(odometry_msg->header.stamp)) ||
    !isFresh(rclcpp::Time(pcd_msg->header.stamp))) {
    return;
  }
  tf2::Transform tf_odom_to_lidar;
  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);

  sensor_msgs::msg::PointCloud2 out;
  pcl_ros::transformPointCloud(lidar_frame_, tf_odom_to_lidar.inverse(), *pcd_msg, out);
  if (isFresh(rclcpp::Time(pcd_msg->header.stamp))) {
    pub_laser_cloud_->publish(out);
  }
}

bool SensorScanGenerationNode::isFresh(const rclcpp::Time & stamp) const
{
  const double age = (now() - stamp).seconds();
  return stamp.nanoseconds() > 0 && age >= -0.05 && age <= max_input_age_;
}

tf2::Transform SensorScanGenerationNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
{
  auto transform_stamped = tf_buffer_->lookupTransform(
    target_frame, source_frame, time, rclcpp::Duration::from_seconds(tf_timeout_));
  tf2::Transform transform;
  tf2::fromMsg(transform_stamped.transform, transform);
  return transform;
}

void SensorScanGenerationNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  br_->sendTransform(transform_msg);
}

void SensorScanGenerationNode::publishOdometry(
  const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  if (previous_odometry_valid_) {
    const double dt = (stamp - previous_odometry_stamp_).seconds();
    if (dt > 0.0) {
      const auto inverse_rotation = transform.getRotation().inverse();
      const auto linear_velocity = tf2::quatRotate(
        inverse_rotation, (transform.getOrigin() - previous_transform_.getOrigin()) / dt);
      auto delta = transform.getRotation() * previous_transform_.getRotation().inverse();
      delta.normalize();
      if (delta.w() < 0.0) {
        delta = tf2::Quaternion(-delta.x(), -delta.y(), -delta.z(), -delta.w());
      }
      const auto angular_velocity =
        tf2::quatRotate(inverse_rotation, delta.getAxis() * (delta.getAngle() / dt));
      out.twist.twist.linear.x = linear_velocity.x();
      out.twist.twist.linear.y = linear_velocity.y();
      out.twist.twist.linear.z = linear_velocity.z();
      out.twist.twist.angular.x = angular_velocity.x();
      out.twist.twist.angular.y = angular_velocity.y();
      out.twist.twist.angular.z = angular_velocity.z();
    }
  }
  previous_transform_ = transform;
  previous_odometry_stamp_ = stamp;
  previous_odometry_valid_ = true;

  pub_chassis_odometry_->publish(out);
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)

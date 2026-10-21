"""Check velocity rotation with delayed odometry and fresh gimbal IMU samples."""
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


def run(use_imu):
    rclpy.init()
    node = rclpy.create_node('heading_check')
    imu_pub = node.create_publisher(Imu, 'gimbal_imu', 100)
    odom_pub = node.create_publisher(Odometry, 'odom', 10)
    cmd_pub = node.create_publisher(Twist, 'input', 10)
    output = []
    subscription = node.create_subscription(TwistStamped, 'output', output.append, 10)
    transforms = []
    tf_subscription = node.create_subscription(TFMessage, '/tf', transforms.append, 10)

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.005)

    def imu(stamp, angle, valid=True):
        msg = Imu()
        msg.header.stamp = rclpy.time.Time(nanoseconds=stamp).to_msg()
        msg.orientation.z = math.sin(angle / 2) if valid else 0.
        msg.orientation.w = math.cos(angle / 2) if valid else 0.
        imu_pub.publish(msg)

    def odom(stamp, angle):
        msg = Odometry()
        msg.header.stamp = rclpy.time.Time(nanoseconds=stamp).to_msg()
        msg.child_frame_id = 'gimbal_yaw'
        msg.pose.pose.orientation.z = math.sin(angle / 2)
        msg.pose.pose.orientation.w = math.cos(angle / 2)
        odom_pub.publish(msg)

    def command():
        count = len(output)
        msg = Twist()
        msg.linear.x = 1.
        msg.angular.z = .2
        cmd_pub.publish(msg)
        until = time.monotonic() + .5
        while len(output) == count and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.005)
        assert len(output) > count, 'Velocity output missing'
        return output[-1]

    with tempfile.TemporaryFile(mode='w+') as log:
        args = [sys.argv[1], '--ros-args', '-p', 'robot_base_frame:=gimbal_yaw',
                '-p', 'fake_robot_base_frame:=gimbal_yaw_fake', '-p', 'odom_topic:=odom',
                '-p', 'input_cmd_vel_topic:=input', '-p', 'output_cmd_vel_topic:=output',
                '-p', 'translation_scale:=-1.0', '-p', 'init_spin_speed:=0.5',
                '-p', 'max_imu_age:=0.1']
        if use_imu:
            args += ['-p', 'gimbal_imu_topic:=gimbal_imu']
        proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        try:
            until = time.monotonic() + 5
            while (cmd_pub.get_subscription_count() == 0 or
                   odom_pub.get_subscription_count() == 0 or
                   node.count_publishers('output') == 0 or
                   (use_imu and imu_pub.get_subscription_count() == 0)):
                assert time.monotonic() < until and proc.poll() is None
                spin(.02)
            for old_yaw, new_yaw in [(1.2, 1.5), (3.1, -3.0), (-3.1, 3.0)]:
                reference = time.time_ns() - 180_000_000
                # Interpolation covers a different absolute IMU world origin.
                imu(reference - 10_000_000, old_yaw - .01)
                imu(reference + 10_000_000, old_yaw + .01)
                odom(reference, .4)
                until = time.monotonic() + .5
                while not any(
                        rclpy.time.Time.from_msg(tf.header.stamp).nanoseconds == reference
                        for msg in transforms for tf in msg.transforms):
                    assert time.monotonic() < until, 'Odometry transform missing'
                    # Repeat the same sample while DDS discovers the TF subscriber.
                    odom(reference, .4)
                    spin(.005)
                latest = time.time_ns() - 1_000_000
                imu(latest, new_yaw)
                spin(.02)
                msg = command()
                delta = math.atan2(math.sin(new_yaw - old_yaw), math.cos(new_yaw - old_yaw))
                heading = .4 + (delta if use_imu else 0.)
                assert abs(msg.twist.linear.x + math.cos(heading)) < 1e-6
                assert abs(msg.twist.linear.y - math.sin(heading)) < 1e-6
                assert abs(msg.twist.angular.z - .7) < 1e-6
                assert msg.header.frame_id == 'gimbal_yaw'
                if use_imu:
                    assert rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds == latest
                spin(.25)
            if use_imu:
                odom(time.time_ns() - 20_000_000, .4)
                spin(.02)
                msg = command()
                assert msg.twist == Twist(), 'Expired IMU must produce a stop'
                imu(time.time_ns(), 0., valid=False)
                spin(.02)
                assert command().twist == Twist(), 'Invalid IMU must produce a stop'
            odom(time.time_ns() - 1_000_000_000, .4)
            spin(.02)
            assert command().twist == Twist(), 'Expired odometry must produce a stop'
        except BaseException:
            log.seek(0)
            print(log.read()[-2500:])
            raise
        finally:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=5)
            node.destroy_subscription(subscription)
            node.destroy_subscription(tf_subscription)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    os.environ.update(ROS_DOMAIN_ID='94', ROS_LOCALHOST_ONLY='1')
    run(True)
    run(False)
    print('PASS: current heading, angle wrap, source timestamps, input expiry and legacy mode')

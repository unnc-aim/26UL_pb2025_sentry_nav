"""Check remote and behavior spin through the real Hub and velocity converter.

Run with the fake_vel_transform_node executable as the first argument.
All ROS communication uses local domain 96 with synthetic inputs.
"""
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from example_interfaces.msg import Float32
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from universal_controller.msg import UnifiedInput
from chassis_controllers.msg import ChassisControl


def main():
    os.environ.update(ROS_DOMAIN_ID='96', ROS_LOCALHOST_ONLY='1')
    rclpy.init()
    node = rclpy.create_node('remote_spin_check')
    rc_pub = node.create_publisher(UnifiedInput, '/test/rc', 10)
    odom_pub = node.create_publisher(Odometry, '/test/odom', 10)
    cmd_pub = node.create_publisher(Twist, '/test/input', 10)
    output, chassis, remote = [], [], []
    node.create_subscription(TwistStamped, '/test/nav', output.append, 10)
    node.create_subscription(ChassisControl, '/chassis_command', chassis.append,
                             qos_profile_sensor_data)
    node.create_subscription(Float32, '/universal_controller/remote_spin', remote.append, 10)
    rc = UnifiedInput(connected=True, emergency_stop=False, navigation_enabled=True,
                      spin_mode=True, wz=3000.)
    command = Twist()
    command.linear.x = .15
    command.angular.z = .2
    spin_pub = None

    def pump(seconds, send_rc=True, fresh_odom=True):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            stamp = node.get_clock().now().to_msg()
            if send_rc:
                rc.header.stamp = stamp
                rc_pub.publish(rc)
            if fresh_odom:
                odom = Odometry(child_frame_id='gimbal_yaw')
                odom.header.stamp = stamp
                odom.pose.pose.orientation.w = 1.
                odom_pub.publish(odom)
            cmd_pub.publish(command)
            end = time.monotonic() + .02
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.002)

    def check_spin(expected):
        assert len(output) > 5 and len(chassis) > 5 and remote
        assert all(abs(m.twist.angular.z - expected) < 1e-5 for m in output[-5:]), output[-1]
        assert all(m.emergency_stop == 0 and abs(m.spin_speed - expected) < 1e-5
                   for m in chassis[-5:]), chassis[-1]

    hub = Path(get_package_prefix('universal_controller')) / 'lib/universal_controller/universal_controller_node'
    hub_config = Path(get_package_share_directory('universal_controller')) / 'config/controller_params.sentry.yaml'
    processes = []
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            processes.append(subprocess.Popen([
                str(hub), '--ros-args', '--params-file', str(hub_config),
                '-p', 'rc_hub.topic_ndj_input:=/test/rc',
                '-p', 'topics.nav_cmd_vel:=/test/nav', '-p', 'topics.cmd_spin:=/test/spin',
                '-p', 'control_frequency:=100'], stdout=log, stderr=subprocess.STDOUT))
            processes.append(subprocess.Popen([
                sys.argv[1], '--ros-args', '-p', 'robot_base_frame:=gimbal_yaw',
                '-p', 'odom_topic:=/test/odom', '-p', 'input_cmd_vel_topic:=/test/input',
                '-p', 'output_cmd_vel_topic:=/test/nav', '-p', 'cmd_spin_topic:=/test/spin',
                '-p', 'remote_spin_topic:=/universal_controller/remote_spin'],
                stdout=log, stderr=subprocess.STDOUT))
            pump(2.)
            assert all(p.poll() is None for p in processes)
            check_spin(.2 + 3000. / 1348.)
            rc.spin_mode = False
            pump(.4)
            check_spin(.2)
            rc.spin_mode = True
            spin_pub = node.create_publisher(Float32, '/test/spin', 10)
            pump(.4)
            check_spin(.2)
            spin_pub.publish(Float32(data=0.))
            command = Twist()  # Initialization/wait-game heartbeat.
            pump(.4)
            check_spin(0.)
            command.linear.x = .15
            command.angular.z = .2
            spin_pub.publish(Float32(data=2.2))
            pump(.4)
            check_spin(2.4)
            node.destroy_publisher(spin_pub)
            spin_pub = None
            pump(.6)
            check_spin(.2 + 3000. / 1348.)
            rc.emergency_stop = True
            pump(.4)
            assert remote[-1].data == 0. and chassis[-1].emergency_stop == 1
            rc.emergency_stop = False
            pump(.4)
            check_spin(.2 + 3000. / 1348.)
            pump(.8, send_rc=False)
            assert remote[-1].data == 0. and chassis[-1].emergency_stop == 1
            pump(.4)
            pump(.8, fresh_odom=False)
            check_spin(0.)
            assert output[-1].twist == Twist()
            pump(.4)
            processes[0].send_signal(signal.SIGINT)
            processes[0].wait(timeout=5)
            pump(.4)
            assert abs(output[-1].twist.angular.z - .2) < 1e-5
            print('PASS: RC spin, switch off, tree zero, tree spin once, tree exit, emergency, RC expiry, odometry expiry, remote expiry')
        except BaseException:
            log.seek(0)
            print(log.read()[-3000:])
            raise
        finally:
            for process in processes:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

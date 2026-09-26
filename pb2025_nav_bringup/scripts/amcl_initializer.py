#!/usr/bin/env python3
"""Initialize native AMCL and report fresh localization data."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.logging import get_logging_directory
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def aligned_pair(poses, scans, tolerance=.025):
    """Select the newest pose with a scan at the same measurement time."""
    scans = list(scans)
    if not scans:
        return None
    for pose in reversed(list(poses)):
        def delta(scan):
            return (scan.header.stamp.sec - pose.header.stamp.sec) + (
                scan.header.stamp.nanosec - pose.header.stamp.nanosec) * 1e-9
        scan = min(scans, key=lambda item: abs(delta(item)))
        if abs(delta(scan)) <= tolerance:
            return pose, scan
    return None


def pose_status(pose, scan, now, initialized_at):
    stamp = Time.from_msg(pose.header.stamp).nanoseconds * 1e-9
    scan_stamp = Time.from_msg(scan.header.stamp).nanoseconds * 1e-9
    p, q = pose.pose.pose.position, pose.pose.pose.orientation
    covariance = [pose.pose.covariance[i] for i in (0, 7, 35)]
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, *covariance]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Localization values must be finite')
    if abs(sum(value * value for value in (q.x, q.y, q.z, q.w)) - 1.) > .01:
        raise ValueError('Localization orientation must have unit length')
    fresh = (stamp >= initialized_at and -.05 <= now - stamp <= .75 and
             -.05 <= now - scan_stamp <= .75 and abs(stamp - scan_stamp) <= .025)
    accepted = (fresh and pose.header.frame_id == 'map' and
                scan.header.frame_id == 'base_footprint' and
                all(0 <= value < limit for value, limit in zip(covariance, (.04, .04, .025))) and
                any(math.isfinite(r) and scan.range_min <= r <= scan.range_max for r in scan.ranges))
    return accepted, dict(pose=[p.x, p.y, yaw(q)], covariance=covariance,
                         pose_age_s=now - stamp, scan_age_s=now - scan_stamp,
                         pose_scan_offset_s=stamp - scan_stamp)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--globalize', action='store_true')
    parser.add_argument(
        '--output', type=Path,
        default=Path(get_logging_directory()) / 'global_localization.json')
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be positive and finite')
    rclpy.init(args=ros_args)
    node = rclpy.create_node('acceptance_localization_probe')
    scans, poses, maps = deque(maxlen=200), deque(maxlen=100), deque(maxlen=1)
    node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', poses.append, 10)
    node.create_subscription(LaserScan, 'obstacle_scan', scans.append, qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, 'map', maps.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(Bool, 'acceptance/localization_ready', 1)
    globalize = node.create_client(Empty, 'reinitialize_global_localization')
    nomotion = node.create_client(Empty, 'request_nomotion_update')
    amcl_state = node.create_client(GetState, 'amcl/get_state')
    report = {'motion_authorized': False, 'globalization_requested': False, 'trace': []}
    initialized_at = node.get_clock().now().nanoseconds * 1e-9

    def save(row):
        report['latest'] = row
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix('.tmp')
        temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        temporary.replace(args.output)
        publisher.publish(Bool(data=row['candidate_stable']))

    def response(future, seconds=5):
        until = time.monotonic() + seconds
        while rclpy.ok() and not future.done() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        if not future.done():
            raise RuntimeError('Localization service response timed out')
        return future.result()

    def fresh_transforms(include_map):
        now = node.get_clock().now().nanoseconds * 1e-9
        frames = [('odom', 'base_footprint'), ('base_footprint', 'gimbal_yaw')]
        if include_map:
            frames.append(('map', 'base_footprint'))
        for parent, child in frames:
            tf = buffer.lookup_transform(parent, child, Time())
            age = now - Time.from_msg(tf.header.stamp).nanoseconds * 1e-9
            if not -.05 <= age <= .75:
                raise ValueError(f'TF {parent}/{child} age {age:.3f} s')

    failed = False
    try:
        if args.globalize:
            deadline = time.monotonic() + 60
            while True:
                save(dict(time=time.time(), candidate_stable=False, reason='Waiting for AMCL inputs'))
                rclpy.spin_once(node, timeout_sec=.1)
                if time.monotonic() >= deadline:
                    raise RuntimeError('AMCL startup timed out')
                if not maps or not scans or not globalize.service_is_ready() or not amcl_state.service_is_ready():
                    continue
                now = node.get_clock().now().nanoseconds * 1e-9
                scan = scans[-1]
                if (maps[-1].header.frame_id != 'map' or 0 not in maps[-1].data or
                    not -.05 <= now - Time.from_msg(scan.header.stamp).nanoseconds * 1e-9 <= .75 or
                    not any(math.isfinite(r) and scan.range_min <= r <= scan.range_max for r in scan.ranges)):
                    continue
                try:
                    fresh_transforms(False)
                    buffer.lookup_transform('base_footprint', scan.header.frame_id, Time.from_msg(scan.header.stamp))
                except Exception:
                    continue
                if response(amcl_state.call_async(GetState.Request())).current_state.id == State.PRIMARY_STATE_ACTIVE:
                    break
            response(globalize.call_async(Empty.Request()))
            poses.clear()
            initialized_at = node.get_clock().now().nanoseconds * 1e-9
            report['globalization_requested'] = True
        end = time.monotonic() + args.seconds
        next_sample, next_update, stable_since = 0., 0., None
        pending_update = None
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            steady_now = time.monotonic()
            if pending_update is not None and pending_update.done():
                pending_update.result()
                pending_update = None
            if pending_update is None and steady_now >= next_update and nomotion.service_is_ready():
                pending_update = nomotion.call_async(Empty.Request())
                next_update = steady_now + .5
            if steady_now < next_sample:
                continue
            next_sample = steady_now + .5
            row = dict(time=time.time(), candidate_stable=False)
            try:
                pair = aligned_pair(poses, scans)
                if pair is None:
                    raise ValueError('Waiting for matching pose and scan times')
                now = node.get_clock().now().nanoseconds * 1e-9
                good, values = pose_status(*pair, now, initialized_at)
                row.update(values)
                fresh_transforms(True)
                stable_since = (steady_now if stable_since is None else stable_since) if good else None
                row['candidate_stable'] = stable_since is not None and steady_now - stable_since >= 3
            except Exception as exc:
                row['reason'] = str(exc)
                stable_since = None
            report['trace'].append(row)
            save(row)
    except (Exception, KeyboardInterrupt) as exc:
        failed = True
        report['error'] = str(exc) or 'Interrupted'
    finally:
        save(dict(time=time.time(), candidate_stable=False, reason=report.get('error', 'Probe stopped')))
        print(json.dumps({key: value for key, value in report.items() if key != 'trace'}))
        node.destroy_node()
        rclpy.try_shutdown()
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())

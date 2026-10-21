#!/usr/bin/env python3
"""Initialize native AMCL and report fresh localization data."""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time
from threading import Event

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.logging import get_logging_directory
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener


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
    parser.add_argument('--startup-timeout', type=float, default=60)
    parser.add_argument('--search-timeout', type=float, default=60)
    parser.add_argument('--verification-timeout', type=float, default=20)
    parser.add_argument('--globalize', action='store_true')
    parser.add_argument('--method', choices=('amcl', 'scan_match'), default='amcl')
    parser.add_argument(
        '--output', type=Path,
        default=Path(get_logging_directory()) / 'global_localization.json')
    args, ros_args = parser.parse_known_args()
    if any(not math.isfinite(value) or value <= 0 for value in
           (args.seconds, args.startup_timeout, args.search_timeout, args.verification_timeout)):
        parser.error('Time limits must be positive and finite')
    relocation_deadline = time.monotonic() + min(args.search_timeout, 60.0)
    rclpy.init(args=ros_args)
    node = rclpy.create_node('acceptance_localization_probe')
    scans, poses, maps = deque(maxlen=200), deque(maxlen=100), deque(maxlen=1)
    node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', poses.append, 10)
    node.create_subscription(LaserScan, 'obstacle_scan', scans.append, qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, 'map', maps.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    buffer = Buffer(cache_time=Duration(seconds=60))
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(Bool, 'acceptance/localization_ready', 1)
    globalize = node.create_client(Empty, 'reinitialize_global_localization')
    nomotion = node.create_client(Empty, 'request_nomotion_update')
    amcl_state = node.create_client(GetState, 'amcl/get_state')
    report = {'motion_authorized': False, 'globalization_requested': False,
              'initialization_method': args.method, 'trace': []}
    initialized_at = node.get_clock().now().nanoseconds * 1e-9

    def save(row):
        report['latest'] = row
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix('.tmp')
        temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        temporary.replace(args.output)
        publisher.publish(Bool(data=row['candidate_stable']))

    def response(future, seconds=5, deadline=None):
        until = time.monotonic() + seconds
        if deadline is not None:
            until = min(until, deadline)
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

    def initialize_from_scan(deadline):
        from scan_map_matcher import ScanMapMatcher

        min_score = node.declare_parameter('search_min_score', 0.85).value
        min_margin = node.declare_parameter('search_min_margin', 0.08).value
        xy_step = node.declare_parameter('search_xy_step', 0.2).value
        angle_step = node.declare_parameter('search_angle_step_degrees', 3.0).value
        initial_pose = node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        cancel = Event()
        pool = ThreadPoolExecutor(max_workers=1)

        def odom_pose(scan):
            tf = buffer.lookup_transform(
                'odom', 'base_footprint', Time.from_msg(scan.header.stamp)).transform
            return [tf.translation.x, tf.translation.y, yaw(tf.rotation)]

        def current_scan():
            fresh_transforms(False)
            for scan in reversed(scans):
                stamp = Time.from_msg(scan.header.stamp)
                age = (node.get_clock().now() - stamp).nanoseconds * 1e-9
                if (scan.header.frame_id == 'base_footprint' and -.05 <= age <= .75
                        and buffer.can_transform('odom', 'base_footprint', stamp)):
                    return scan
            raise ValueError('Waiting for a fresh scan with odometry at its measurement time')

        def recent_views(matcher, scan, reference_odom):
            views, ages = [], []
            scan_time = Time.from_msg(scan.header.stamp)
            for previous in list(scans):
                age = (scan_time - Time.from_msg(previous.header.stamp)).nanoseconds * 1e-9
                if previous.header.frame_id != 'base_footprint' or not 0 <= age <= 10:
                    continue
                try:
                    previous_odom = odom_pose(previous)
                    if math.hypot(previous_odom[0] - reference_odom[0],
                                  previous_odom[1] - reference_odom[1]) > .1:
                        continue
                    views.append((matcher.scan_points(previous, max_points=None), previous_odom))
                    ages.append(age)
                except (ValueError, TransformException):
                    continue
            return views, ages

        try:
            while rclpy.ok() and time.monotonic() < deadline:
                try:
                    import numpy as np

                    scan = current_scan()
                    reference_odom = odom_pose(scan)
                    map_msg = maps[-1]
                    info = map_msg.info
                    orientation = info.origin.orientation
                    if (map_msg.header.frame_id != 'map'
                            or abs(sum(v * v for v in (orientation.x, orientation.y,
                                                      orientation.z, orientation.w)) - 1) > .01):
                        raise ValueError('Map frame and origin orientation must be valid')
                    matcher = ScanMapMatcher(
                        np.asarray(map_msg.data).reshape(info.height, info.width), info.resolution,
                        [info.origin.position.x, info.origin.position.y, yaw(orientation)],
                        xy_step, math.radians(angle_step), min_score, min_margin)
                    points = matcher.scan_points(scan)
                    views, ages = recent_views(matcher, scan, reference_odom)
                    coverage = len(points) * .5
                    if views:
                        points, coverage = matcher.combine_views(views, reference_odom)
                    started = time.monotonic()
                    future = pool.submit(matcher.search, points, cancel)
                    node.get_logger().info('Searching the full map from the current scan')
                    save(dict(time=time.time(), candidate_stable=False, reason='Searching full map'))
                    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=.05)
                    if not rclpy.ok() or not future.done():
                        raise RuntimeError('Full-map search timed out or was interrupted')
                    candidates = future.result()
                    report['scan_search'] = dict(
                        seconds=time.monotonic() - started,
                        scan_stamp=Time.from_msg(scan.header.stamp).nanoseconds * 1e-9,
                        source_scans=len(views), source_span_s=max(ages, default=0),
                        angular_coverage_degrees=coverage,
                        finite_rays=len(points), candidates=candidates)
                    attempt = dict(seconds=report['scan_search']['seconds'],
                                   scan_stamp=report['scan_search']['scan_stamp'],
                                   source_scans=len(views), source_span_s=max(ages, default=0),
                                   angular_coverage_degrees=coverage,
                                   candidates=candidates[:2])
                    report.setdefault('search_attempts', []).append(attempt)
                    if not candidates or not matcher.supported(candidates[0]):
                        attempt['reason'] = 'Search view has insufficient map agreement'
                        raise ValueError(attempt['reason'])
                    node.get_logger().info('Comparing candidate poses with later gimbal views')
                    # Retain competing poses while the gimbal supplies another view.
                    last_stamp = Time.from_msg(scan.header.stamp).nanoseconds
                    previous_odom = None
                    verified = 0
                    previous_candidate = None
                    verification_deadline = min(deadline, time.monotonic() + args.verification_timeout)
                    while rclpy.ok() and time.monotonic() < verification_deadline:
                        rclpy.spin_once(node, timeout_sec=.05)
                        fresh = current_scan()
                        stamp = Time.from_msg(fresh.header.stamp).nanoseconds
                        if stamp <= last_stamp:
                            continue
                        dt = (stamp - last_stamp) * 1e-9
                        last_stamp = stamp
                        if maps[-1] is not map_msg:
                            raise ValueError('Map changed during search; restarting')
                        current_odom = odom_pose(fresh)
                        if previous_odom is not None:
                            distance = math.hypot(current_odom[0] - previous_odom[0],
                                                  current_odom[1] - previous_odom[1])
                            angle = math.atan2(math.sin(current_odom[2] - previous_odom[2]),
                                               math.cos(current_odom[2] - previous_odom[2]))
                            if distance / dt > .1 or abs(angle) / dt > .15:
                                previous_odom = current_odom
                                verified = 0
                                previous_candidate = None
                                save(dict(time=time.time(), candidate_stable=False,
                                          reason='Waiting for a stationary base during pose verification'))
                                continue
                        previous_odom = current_odom
                        views, _ = recent_views(matcher, fresh, current_odom)
                        checked = matcher.observe(
                            matcher.scan_points(fresh), candidates, reference_odom, current_odom,
                            views=views)
                        attempt['verification_source_scans'] = len(views)
                        attempt['latest_verification'] = checked[:2]
                        report['scan_search']['latest_verification'] = checked[:2]
                        if not matcher.accepted(checked):
                            verified = 0
                            previous_candidate = None
                            save(dict(time=time.time(), candidate_stable=False,
                                      reason='Waiting for a distinguishing scan view'))
                            continue
                        candidate = checked[0]['candidate_index']
                        verified = verified + 1 if candidate == previous_candidate else 1
                        previous_candidate = candidate
                        if verified < 3 or initial_pose.get_subscription_count() == 0:
                            continue
                        # Keep the measurement timestamp after consecutive near-stationary observations.
                        age = (node.get_clock().now() - Time.from_msg(fresh.header.stamp)).nanoseconds * 1e-9
                        if not -.05 <= age <= .75:
                            raise ValueError('Verification scan expired; restarting')
                        seed = PoseWithCovarianceStamped()
                        seed.header.frame_id = 'map'
                        seed.header.stamp = fresh.header.stamp
                        x, y, angle = checked[0]['pose']
                        seed.pose.pose.position.x, seed.pose.pose.position.y = float(x), float(y)
                        seed.pose.pose.orientation.z = math.sin(angle / 2)
                        seed.pose.pose.orientation.w = math.cos(angle / 2)
                        seed.pose.covariance[0] = seed.pose.covariance[7] = .04
                        seed.pose.covariance[35] = .02
                        if time.monotonic() >= deadline:
                            raise RuntimeError('Full-map initialization timed out')
                        initial_pose.publish(seed)
                        report['initial_pose_published'] = True
                        report['scan_search'].update(verification_scans=verified, verified=checked[:2])
                        attempt.update(verification_scans=verified, verified=checked[:2])
                        node.get_logger().info(
                            f'Full-map pose sent to AMCL: x={x:.3f}, y={y:.3f}, yaw={angle:.3f}')
                        return
                    attempt['reason'] = 'Views remained ambiguous or inconsistent; starting another search'
                    raise ValueError(attempt['reason'])
                except (ValueError, TransformException) as exc:
                    save(dict(time=time.time(), candidate_stable=False, reason=str(exc)))
                    node.get_logger().warning(str(exc), throttle_duration_sec=2.0)
                    until = min(deadline, time.monotonic() + 1)
                    while rclpy.ok() and time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=.05)
            raise RuntimeError('Full-map initialization timed out')
        finally:
            cancel.set()
            pool.shutdown(wait=True, cancel_futures=True)

    failed = False
    try:
        if args.globalize:
            deadline = min(relocation_deadline, time.monotonic() + args.startup_timeout)
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
                if response(amcl_state.call_async(GetState.Request()), deadline=deadline).current_state.id == State.PRIMARY_STATE_ACTIVE:
                    break
            if args.method == 'scan_match':
                initialize_from_scan(relocation_deadline)
            else:
                response(globalize.call_async(Empty.Request()), deadline=relocation_deadline)
                report['globalization_requested'] = True
            poses.clear()
            initialized_at = node.get_clock().now().nanoseconds * 1e-9
        end = time.monotonic() + args.seconds
        if args.globalize:
            end = min(end, relocation_deadline)
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

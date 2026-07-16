#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""지도작성 시험을 CSV/JSON/PGM으로 기록하는 ROS1 노드."""

import csv
import datetime
import json
import math
import os
import socket
import statistics
import threading
import time

import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

try:
    from puppy_vr_control_noetic.mapping_artifacts import (
        angle_delta, occupancy_counts, quaternion_to_yaw, write_map_yaml, write_pgm)
except ImportError:
    import sys
    source_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
    if source_dir not in sys.path:
        sys.path.insert(0, source_dir)
    from puppy_vr_control_noetic.mapping_artifacts import (
        angle_delta, occupancy_counts, quaternion_to_yaw, write_map_yaml, write_pgm)


def utc_now():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + 'Z'


class MappingRunRecorder(object):

    def __init__(self):
        rospy.init_node('mapping_run_recorder')

        self.output_dir = os.path.abspath(os.path.expanduser(
            rospy.get_param('~output_dir')))
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.pose_topic = rospy.get_param('~pose_topic', '/lidar_mapping/pose')
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.save_service = rospy.get_param(
            '~save_service', '/lidar_mapping/save_map')
        self.duration_sec = float(rospy.get_param('~duration_sec', 120.0))
        self.stationary_sec = float(rospy.get_param('~stationary_sec', 20.0))
        self.snapshot_period_sec = float(
            rospy.get_param('~snapshot_period_sec', 5.0))
        self.expect_return = bool(rospy.get_param('~expect_return', False))
        self.filter_min_range = float(rospy.get_param('~filter_min_range', 0.15))
        self.filter_max_range = float(rospy.get_param('~filter_max_range', 8.0))
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')

        self.maps_dir = os.path.join(self.output_dir, 'maps')
        os.makedirs(self.maps_dir, exist_ok=True)
        self.lock = threading.RLock()
        self.finalized = False
        self.started_at = utc_now()
        self.started_monotonic = time.monotonic()

        self.scan_count = 0
        self.scan_valid_total = 0
        self.scan_min_observed = None
        self.scan_metadata = None
        self.last_scan_elapsed = None
        self.max_scan_gap = 0.0
        self.scan_gaps = []
        self.map_count = 0
        self.last_map_elapsed = None
        self.max_map_gap = 0.0
        self.map_gaps = []
        self.known_decrease_events = 0
        self.max_known_cell_drop = 0
        self.map_metadata_changes = 0
        self.pose_count = 0
        self.last_pose_elapsed = None
        self.max_pose_gap = 0.0
        self.pose_gaps = []
        self.min_map_boundary_margin = None
        self.snapshot_count = 0
        self.last_snapshot_elapsed = None
        self.previous_map = None
        self.latest_map = None
        self.latest_map_info = None
        self.initial_map_counts = None
        self.final_map_counts = None

        self.first_pose = None
        self.last_pose = None
        self.final_pose = None
        self.pose_path_length = 0.0
        self.max_pose_step = 0.0
        self.max_yaw_step = 0.0
        self.stationary_origin = None
        self.stationary_max_drift = 0.0
        self.stationary_max_yaw_drift = 0.0

        self.pose_file, self.pose_writer = self._csv_file(
            'pose.csv', ['elapsed_sec', 'ros_time', 'x', 'y', 'yaw'])
        self.scan_file, self.scan_writer = self._csv_file(
            'scan_metrics.csv',
            ['elapsed_sec', 'ros_time', 'beam_count', 'valid_count',
             'min_range', 'median_range', 'max_range'])
        self.map_file, self.map_writer = self._csv_file(
            'map_metrics.csv',
            ['elapsed_sec', 'ros_time', 'width', 'height', 'resolution',
             'unknown', 'free', 'occupied', 'uncertain', 'known', 'changed_cells'])

        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_cb, queue_size=20)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_cb, queue_size=2)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb, queue_size=20)
        rospy.on_shutdown(self.finalize)
        if self.duration_sec > 0:
            threading.Thread(target=self.wall_timeout, daemon=True).start()

        rospy.loginfo(
            '지도 검증 기록 시작: %.1fs, 정지 구간 %.1fs, 출력=%s',
            self.duration_sec, self.stationary_sec, self.output_dir)

    def _csv_file(self, name, fields):
        stream = open(os.path.join(self.output_dir, name), 'w', newline='', buffering=1)
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        return stream, writer

    def elapsed(self):
        return time.monotonic() - self.started_monotonic

    @staticmethod
    def stamp_seconds(header):
        stamp = header.stamp.to_sec()
        return stamp if stamp > 0 else rospy.Time.now().to_sec()

    def scan_cb(self, msg):
        elapsed = self.elapsed()
        lower = max(float(msg.range_min), self.filter_min_range)
        upper = min(float(msg.range_max), self.filter_max_range)
        valid = [value for value in msg.ranges
                 if math.isfinite(value) and lower <= value <= upper]
        minimum = min(valid) if valid else ''
        maximum = max(valid) if valid else ''
        median = statistics.median(valid) if valid else ''
        with self.lock:
            if self.finalized:
                return
            self.scan_count += 1
            if self.last_scan_elapsed is not None:
                gap = elapsed - self.last_scan_elapsed
                self.scan_gaps.append(gap)
                self.max_scan_gap = max(self.max_scan_gap, gap)
            self.last_scan_elapsed = elapsed
            self.scan_valid_total += len(valid)
            if self.scan_metadata is None:
                self.scan_metadata = {
                    'frame_id': msg.header.frame_id,
                    'beam_count': len(msg.ranges),
                    'angle_min': float(msg.angle_min),
                    'angle_max': float(msg.angle_max),
                    'angle_increment': float(msg.angle_increment),
                    'message_range_min': float(msg.range_min),
                    'message_range_max': float(msg.range_max),
                    'filter_min_range': self.filter_min_range,
                    'filter_max_range': self.filter_max_range,
                }
            if valid:
                self.scan_min_observed = minimum if self.scan_min_observed is None \
                    else min(self.scan_min_observed, minimum)
            self.scan_writer.writerow({
                'elapsed_sec': '%.6f' % elapsed,
                'ros_time': '%.9f' % self.stamp_seconds(msg.header),
                'beam_count': len(msg.ranges),
                'valid_count': len(valid),
                'min_range': minimum,
                'median_range': median,
                'max_range': maximum,
            })

    def pose_cb(self, msg):
        elapsed = self.elapsed()
        q = msg.pose.orientation
        pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            quaternion_to_yaw(q.x, q.y, q.z, q.w),
        )
        with self.lock:
            if self.finalized:
                return
            self.pose_count += 1
            if self.last_pose_elapsed is not None:
                gap = elapsed - self.last_pose_elapsed
                self.pose_gaps.append(gap)
                self.max_pose_gap = max(self.max_pose_gap, gap)
            self.last_pose_elapsed = elapsed
            if self.first_pose is None:
                self.first_pose = pose
            if self.last_pose is not None:
                step = math.hypot(
                    pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
                yaw_step = abs(angle_delta(pose[2], self.last_pose[2]))
                self.pose_path_length += step
                self.max_pose_step = max(self.max_pose_step, step)
                self.max_yaw_step = max(self.max_yaw_step, yaw_step)
            if elapsed <= self.stationary_sec:
                if self.stationary_origin is None:
                    self.stationary_origin = pose
                self.stationary_max_drift = max(
                    self.stationary_max_drift,
                    math.hypot(pose[0] - self.stationary_origin[0],
                               pose[1] - self.stationary_origin[1]))
                self.stationary_max_yaw_drift = max(
                    self.stationary_max_yaw_drift,
                    abs(angle_delta(pose[2], self.stationary_origin[2])))
            self.last_pose = pose
            self.final_pose = pose
            if self.latest_map_info is not None:
                info = self.latest_map_info
                max_x = info['origin_x'] + info['width'] * info['resolution']
                max_y = info['origin_y'] + info['height'] * info['resolution']
                margin = min(
                    pose[0] - info['origin_x'], max_x - pose[0],
                    pose[1] - info['origin_y'], max_y - pose[1])
                self.min_map_boundary_margin = margin if \
                    self.min_map_boundary_margin is None else min(
                        self.min_map_boundary_margin, margin)
            self.pose_writer.writerow({
                'elapsed_sec': '%.6f' % elapsed,
                'ros_time': '%.9f' % self.stamp_seconds(msg.header),
                'x': '%.8f' % pose[0],
                'y': '%.8f' % pose[1],
                'yaw': '%.8f' % pose[2],
            })

    def map_cb(self, msg):
        elapsed = self.elapsed()
        data = list(msg.data)
        counts = occupancy_counts(data)
        with self.lock:
            if self.finalized:
                return
            self.map_count += 1
            if self.last_map_elapsed is not None:
                gap = elapsed - self.last_map_elapsed
                self.map_gaps.append(gap)
                self.max_map_gap = max(self.max_map_gap, gap)
            self.last_map_elapsed = elapsed
            changed = -1 if self.previous_map is None else sum(
                1 for old, new in zip(self.previous_map, data) if old != new)
            if self.final_map_counts is not None:
                drop = self.final_map_counts['known'] - counts['known']
                if drop > 0:
                    self.known_decrease_events += 1
                    self.max_known_cell_drop = max(self.max_known_cell_drop, drop)
            self.previous_map = data
            self.latest_map = data
            new_map_info = {
                'width': int(msg.info.width),
                'height': int(msg.info.height),
                'resolution': float(msg.info.resolution),
                'origin_x': float(msg.info.origin.position.x),
                'origin_y': float(msg.info.origin.position.y),
                'frame_id': msg.header.frame_id or 'map',
            }
            if self.latest_map_info is not None and new_map_info != self.latest_map_info:
                self.map_metadata_changes += 1
            self.latest_map_info = new_map_info
            if self.initial_map_counts is None:
                self.initial_map_counts = dict(counts)
            self.final_map_counts = dict(counts)
            self.map_writer.writerow({
                'elapsed_sec': '%.6f' % elapsed,
                'ros_time': '%.9f' % self.stamp_seconds(msg.header),
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'unknown': counts['unknown'],
                'free': counts['free'],
                'occupied': counts['occupied'],
                'uncertain': counts['uncertain'],
                'known': counts['known'],
                'changed_cells': changed,
            })
            if (self.last_snapshot_elapsed is None or
                    elapsed - self.last_snapshot_elapsed >= self.snapshot_period_sec):
                self._write_snapshot(data, self.latest_map_info)
                self.last_snapshot_elapsed = elapsed

    def _write_snapshot(self, data, info):
        self.snapshot_count += 1
        path = os.path.join(
            self.maps_dir, 'map_%04d.pgm' % self.snapshot_count)
        write_pgm(path, data, info['width'], info['height'])

    def wall_timeout(self):
        """ROS simulated time과 무관하게 지정된 실제 시간이 지나면 종료한다."""
        deadline = self.started_monotonic + self.duration_sec
        while not rospy.is_shutdown():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(0.5, remaining))
        if not rospy.is_shutdown():
            self.finalize()
            rospy.signal_shutdown('지도 검증 수집 시간 종료')

    def _request_map_save(self):
        try:
            rospy.wait_for_service(self.save_service, timeout=3.0)
            response = rospy.ServiceProxy(self.save_service, Trigger)()
            return bool(response.success), response.message
        except Exception as exc:
            return False, str(exc)

    @staticmethod
    def _check(checks, name, status, detail):
        checks.append({'name': name, 'status': status, 'detail': detail})

    @staticmethod
    def _percentile(values, percentile):
        if not values:
            return 0.0
        ordered = sorted(values)
        index = int(math.ceil((percentile / 100.0) * len(ordered))) - 1
        return ordered[max(0, min(len(ordered) - 1, index))]

    def _make_summary(self, save_success, save_message):
        duration = max(0.001, self.elapsed())
        average_valid = (float(self.scan_valid_total) / self.scan_count
                         if self.scan_count else 0.0)
        scan_p95_gap = self._percentile(self.scan_gaps, 95.0)
        map_p95_gap = self._percentile(self.map_gaps, 95.0)
        pose_p95_gap = self._percentile(self.pose_gaps, 95.0)
        checks = []

        self._check(checks, 'scan_messages',
                    'PASS' if self.scan_count else 'FAIL',
                    '%d messages, %.2f Hz' %
                    (self.scan_count, self.scan_count / duration))
        self._check(checks, 'scan_valid_beams',
                    'PASS' if average_valid >= 30 else 'FAIL',
                    'average %.1f valid beams' % average_valid)
        self._check(checks, 'scan_max_gap',
                    'PASS' if self.max_scan_gap <= 1.0 else 'WARN',
                    'p95 %.3f s, maximum %.3f s' %
                    (scan_p95_gap, self.max_scan_gap))
        self._check(checks, 'map_messages',
                    'PASS' if self.map_count else 'FAIL',
                    '%d messages, %.2f Hz' %
                    (self.map_count, self.map_count / duration))
        self._check(checks, 'map_max_gap',
                    'PASS' if self.max_map_gap <= 2.5 else 'WARN',
                    'p95 %.3f s, maximum %.3f s' %
                    (map_p95_gap, self.max_map_gap))
        self._check(checks, 'pose_messages',
                    'PASS' if self.pose_count else 'FAIL',
                    '%d messages, %.2f Hz' %
                    (self.pose_count, self.pose_count / duration))
        self._check(checks, 'pose_max_gap',
                    'PASS' if self.max_pose_gap <= 1.0 else 'WARN',
                    'p95 %.3f s, maximum %.3f s' %
                    (pose_p95_gap, self.max_pose_gap))
        pose_scan_ratio = (float(self.pose_count) / self.scan_count
                           if self.scan_count else 0.0)
        self._check(checks, 'pose_scan_ratio',
                    'PASS' if pose_scan_ratio >= 0.80 else 'WARN',
                    '%.3f pose messages per scan message' % pose_scan_ratio)

        scan_frame = (self.scan_metadata or {}).get('frame_id', '')
        if scan_frame and scan_frame != self.base_frame:
            self._check(
                checks, 'scan_frame', 'WARN',
                '%s differs from %s; verify static TF and LiDAR translation' %
                (scan_frame, self.base_frame))
        else:
            self._check(checks, 'scan_frame', 'PASS',
                        scan_frame or 'frame not reported')

        if self.final_map_counts:
            total = sum(self.final_map_counts[key] for key in
                        ('unknown', 'free', 'occupied', 'uncertain'))
            coverage = 100.0 * self.final_map_counts['known'] / max(1, total)
            growth = (self.final_map_counts['known'] -
                      self.initial_map_counts['known'])
            self._check(checks, 'map_free_space',
                        'PASS' if self.final_map_counts['free'] else 'FAIL',
                        '%d free cells' % self.final_map_counts['free'])
            self._check(checks, 'map_obstacles',
                        'PASS' if self.final_map_counts['occupied'] else 'FAIL',
                        '%d occupied cells' % self.final_map_counts['occupied'])
            self._check(checks, 'map_growth',
                        'PASS' if growth > 0 else 'WARN',
                        '%+d known cells, final coverage %.2f%%' %
                        (growth, coverage))
            self._check(checks, 'known_cells_monotonic',
                        'PASS' if self.known_decrease_events == 0 else 'WARN',
                        '%d decrease events, largest drop %d cells' %
                        (self.known_decrease_events, self.max_known_cell_drop))
        else:
            coverage = 0.0
            growth = 0

        self._check(checks, 'stationary_position_drift',
                    'PASS' if self.stationary_max_drift <= 0.10 else 'WARN',
                    'max %.3f m during first %.1f s' %
                    (self.stationary_max_drift, self.stationary_sec))
        self._check(checks, 'stationary_yaw_drift',
                    'PASS' if self.stationary_max_yaw_drift <= math.radians(5.0)
                    else 'WARN',
                    'max %.2f deg during first %.1f s' %
                    (math.degrees(self.stationary_max_yaw_drift), self.stationary_sec))
        self._check(checks, 'pose_step',
                    'PASS' if self.max_pose_step <= 0.30 else 'WARN',
                    'max position step %.3f m' % self.max_pose_step)
        if self.min_map_boundary_margin is not None:
            self._check(
                checks, 'map_boundary_margin',
                'PASS' if self.min_map_boundary_margin >= 0.50 else 'WARN',
                'minimum estimated margin %.3f m' % self.min_map_boundary_margin)

        net_displacement = 0.0
        if self.first_pose is not None and self.final_pose is not None:
            net_displacement = math.hypot(
                self.final_pose[0] - self.first_pose[0],
                self.final_pose[1] - self.first_pose[1])
        if self.expect_return:
            self._check(checks, 'return_error',
                        'PASS' if net_displacement <= 0.30 else 'WARN',
                        'start-to-end displacement %.3f m' % net_displacement)

        statuses = [item['status'] for item in checks]
        verdict = 'FAIL' if 'FAIL' in statuses else (
            'WARN' if 'WARN' in statuses else 'PASS')
        return {
            'schema_version': 1,
            'verdict': verdict,
            'started_at': self.started_at,
            'finished_at': utc_now(),
            'duration_sec': duration,
            'host': socket.gethostname(),
            'ros_distro': os.environ.get('ROS_DISTRO', ''),
            'topics': {
                'scan': self.scan_topic,
                'map': self.map_topic,
                'pose': self.pose_topic,
            },
            'configuration': {
                'requested_duration_sec': self.duration_sec,
                'stationary_sec': self.stationary_sec,
                'snapshot_period_sec': self.snapshot_period_sec,
                'expect_return': self.expect_return,
            },
            'messages': {
                'scan': self.scan_count,
                'map': self.map_count,
                'pose': self.pose_count,
                'scan_rate_hz': self.scan_count / duration,
                'map_rate_hz': self.map_count / duration,
                'pose_rate_hz': self.pose_count / duration,
                'average_valid_scan_beams': average_valid,
                'minimum_observed_range_m': self.scan_min_observed,
                'max_scan_gap_sec': self.max_scan_gap,
                'p95_scan_gap_sec': scan_p95_gap,
                'max_map_gap_sec': self.max_map_gap,
                'p95_map_gap_sec': map_p95_gap,
                'max_pose_gap_sec': self.max_pose_gap,
                'p95_pose_gap_sec': pose_p95_gap,
                'pose_scan_ratio': pose_scan_ratio,
            },
            'scan_metadata': self.scan_metadata,
            'map': {
                'initial': self.initial_map_counts,
                'final': self.final_map_counts,
                'known_growth_cells': growth,
                'final_coverage_percent': coverage,
                'snapshots': self.snapshot_count,
                'info': self.latest_map_info,
                'known_decrease_events': self.known_decrease_events,
                'max_known_cell_drop': self.max_known_cell_drop,
                'metadata_changes': self.map_metadata_changes,
            },
            'pose': {
                'path_length_m': self.pose_path_length,
                'net_displacement_m': net_displacement,
                'max_position_step_m': self.max_pose_step,
                'max_yaw_step_deg': math.degrees(self.max_yaw_step),
                'minimum_map_boundary_margin_m': self.min_map_boundary_margin,
                'stationary_max_drift_m': self.stationary_max_drift,
                'stationary_max_yaw_drift_deg': math.degrees(
                    self.stationary_max_yaw_drift),
            },
            'mapper_save_service': {
                'success': save_success,
                'message': save_message,
            },
            'checks': checks,
        }

    def finalize(self):
        with self.lock:
            if self.finalized:
                return
            self.finalized = True
            latest_map = list(self.latest_map) if self.latest_map is not None else None
            latest_info = dict(self.latest_map_info) if self.latest_map_info else None

        save_success, save_message = self._request_map_save()
        if latest_map is not None and latest_info is not None:
            write_pgm(
                os.path.join(self.output_dir, 'final_map.pgm'), latest_map,
                latest_info['width'], latest_info['height'])
            write_map_yaml(
                os.path.join(self.output_dir, 'final_map.yaml'), 'final_map.pgm',
                latest_info['resolution'], latest_info['origin_x'],
                latest_info['origin_y'])

        summary = self._make_summary(save_success, save_message)
        with open(os.path.join(self.output_dir, 'summary.json'), 'w') as stream:
            json.dump(summary, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write('\n')

        for stream in (self.pose_file, self.scan_file, self.map_file):
            stream.flush()
            stream.close()
        rospy.loginfo('지도 검증 기록 완료: %s (%s)',
                      summary['verdict'], self.output_dir)


if __name__ == '__main__':
    MappingRunRecorder()
    rospy.spin()

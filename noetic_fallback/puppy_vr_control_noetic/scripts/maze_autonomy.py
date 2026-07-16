#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LiDAR frontier를 탐색하고 선택적으로 Qwen이 다음 목표를 고르는 ROS1 노드."""

import json
import math
import os
import sys
import threading
import time

import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from puppy_control.msg import Velocity
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse

try:
    from puppy_vr_control_noetic.llm_frontier_selector import LlmFrontierSelector
    from puppy_vr_control_noetic.maze_core import GridMap, choose_lookahead, wrap_angle
except ImportError:
    # catkin 빌드 없이 ROS_PACKAGE_PATH만 등록한 소스 실행도 지원한다.
    source_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
    if source_dir not in sys.path:
        sys.path.insert(0, source_dir)
    from puppy_vr_control_noetic.llm_frontier_selector import LlmFrontierSelector
    from puppy_vr_control_noetic.maze_core import GridMap, choose_lookahead, wrap_angle


class MazeAutonomyNode(object):

    def __init__(self):
        rospy.init_node('maze_autonomy')

        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.pose_topic = rospy.get_param('~pose_topic', '/lidar_mapping/pose')
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.velocity_topic = rospy.get_param(
            '~velocity_topic', '/puppy_control/velocity')
        self.exit_topic = rospy.get_param('~exit_topic', '/maze/exit_detected')

        self.control_rate = float(rospy.get_param('~control_rate', 10.0))
        self.forward_speed = min(float(rospy.get_param('~forward_speed', 6.0)), 12.0)
        self.max_yaw_rate = min(float(rospy.get_param('~max_yaw_rate', 0.28)), 0.6)
        self.turn_gain = float(rospy.get_param('~turn_gain', 1.2))
        self.heading_gain = float(rospy.get_param('~heading_gain', 0.8))
        self.rotate_threshold = float(rospy.get_param('~rotate_threshold', 0.35))
        self.goal_tolerance = float(rospy.get_param('~goal_tolerance', 0.25))
        self.lookahead = float(rospy.get_param('~lookahead', 0.35))
        self.robot_radius = float(rospy.get_param('~robot_radius', 0.18))
        self.stop_distance = float(rospy.get_param('~stop_distance', 0.32))
        self.slow_distance = float(rospy.get_param('~slow_distance', 0.55))
        self.front_cone = math.radians(float(rospy.get_param('~front_cone_deg', 25.0)))
        self.scan_forward_angle = float(rospy.get_param('~scan_forward_angle', 0.0))
        self.blocked_timeout = float(rospy.get_param('~blocked_timeout', 1.5))
        self.stuck_timeout = float(rospy.get_param('~stuck_timeout', 4.0))
        self.replan_period = float(rospy.get_param('~replan_period', 2.0))
        self.data_timeout = float(rospy.get_param('~data_timeout', 2.5))
        self.min_frontier_size = int(rospy.get_param('~min_frontier_size', 3))
        self.max_candidates = int(rospy.get_param('~max_candidates', 5))

        self.use_llm = bool(rospy.get_param('~use_llm', False))
        self.selector = LlmFrontierSelector(
            rospy.get_param('~llama_server_url', 'http://127.0.0.1:8081/completion'),
            rospy.get_param('~llm_timeout_sec', 8.0))

        self.lock = threading.RLock()
        self.grid = None
        self.map_frame = 'map'
        self.pose = None
        self.front_distance = None
        self.map_time = 0.0
        self.pose_time = 0.0
        self.scan_time = 0.0
        self.exit_detected = False

        self.running = bool(rospy.get_param('~autostart', False))
        self.state = 'SELECTING' if self.running else 'IDLE'
        self.goal_cell = None
        self.path = []
        self.failed_goals = []
        self.recent_goal_ids = []
        self.last_plan_time = 0.0
        self.blocked_since = None
        self.last_progress_pose = None
        self.last_progress_time = time.monotonic()
        self.last_status = None

        self.velocity_pub = rospy.Publisher(
            self.velocity_topic, Velocity, queue_size=1)
        self.status_pub = rospy.Publisher('~status', String, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher('~goal', PoseStamped, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('~path', Path, queue_size=1, latch=True)

        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber(self.exit_topic, Bool, self.exit_cb, queue_size=1)
        rospy.Service('~set_running', SetBool, self.set_running)
        rospy.Timer(
            rospy.Duration(1.0 / max(self.control_rate, 1.0)), self.control_tick)
        rospy.on_shutdown(self.shutdown)

        self.publish_status('시작 대기' if not self.running else '자율 탐색 시작')
        rospy.logwarn(
            '자율주행 출력=%s. VR/수동 제어 노드를 동시에 실행하지 마세요.',
            self.velocity_topic)

    def map_cb(self, msg):
        try:
            grid = GridMap(
                msg.info.width, msg.info.height, msg.info.resolution,
                msg.info.origin.position.x, msg.info.origin.position.y, msg.data)
        except ValueError as exc:
            rospy.logwarn_throttle(5, '잘못된 OccupancyGrid: %s', exc)
            return
        with self.lock:
            self.grid = grid
            self.map_frame = msg.header.frame_id or 'map'
            self.map_time = time.monotonic()

    def pose_cb(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.pose = (msg.pose.position.x, msg.pose.position.y, yaw)
            self.pose_time = time.monotonic()

    def scan_cb(self, msg):
        nearest = None
        for index, value in enumerate(msg.ranges):
            if not math.isfinite(value) or value < msg.range_min or value > msg.range_max:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            error = wrap_angle(angle - self.scan_forward_angle)
            if abs(error) <= self.front_cone:
                nearest = value if nearest is None else min(nearest, value)
        with self.lock:
            self.front_distance = nearest
            self.scan_time = time.monotonic()

    def exit_cb(self, msg):
        with self.lock:
            self.exit_detected = bool(msg.data)

    def set_running(self, request):
        with self.lock:
            self.running = bool(request.data)
            self.state = 'SELECTING' if self.running else 'IDLE'
            self.goal_cell = None
            self.path = []
            self.blocked_since = None
            self.last_progress_pose = None
            self.last_progress_time = time.monotonic()
            if request.data:
                self.failed_goals = []
                self.recent_goal_ids = []
        self.publish_stop(repeat=3)
        self.publish_status('자율 탐색 시작' if request.data else '사용자 정지')
        return SetBoolResponse(success=True, message=self.state)

    def publish_status(self, reason):
        payload = {'state': self.state, 'reason': reason, 'llm': self.use_llm}
        text = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        if text != self.last_status:
            self.status_pub.publish(String(data=text))
            self.last_status = text

    def publish_stop(self, repeat=1):
        for _ in range(max(1, int(repeat))):
            self.velocity_pub.publish(Velocity(x=0.0, y=0.0, yaw_rate=0.0))

    def fail_current_goal(self, reason):
        if self.goal_cell is not None:
            self.failed_goals.append(self.goal_cell)
            self.failed_goals = self.failed_goals[-20:]
        self.goal_cell = None
        self.path = []
        self.state = 'SELECTING'
        self.blocked_since = None
        self.publish_stop()
        self.publish_status(reason)

    def select_goal(self, grid, pose):
        self.publish_stop()
        start = grid.world_to_cell(pose[0], pose[1])
        candidates = grid.frontier_candidates(
            start,
            robot_radius=self.robot_radius,
            min_cluster_size=self.min_frontier_size,
            max_candidates=self.max_candidates,
            blacklist=self.failed_goals)
        if not candidates:
            self.running = False
            blocked = grid.inflated_obstacles(self.robot_radius)
            if grid.frontier_cells(blocked):
                self.state = 'BLOCKED'
                self.publish_status('미탐사 영역은 있으나 안전한 경로 없음')
            else:
                self.state = 'COMPLETE'
                self.publish_status('탐색할 미탐사 경계 없음')
            return

        for candidate in candidates:
            gx, gy = candidate['world']
            candidate['bearing_deg'] = math.degrees(
                wrap_angle(math.atan2(gy - pose[1], gx - pose[0]) - pose[2]))

        selected_id = None
        if self.use_llm and len(candidates) > 1:
            try:
                selected_id = self.selector.choose(candidates, self.recent_goal_ids)
                rospy.loginfo('Qwen raw=%s / selected=%s', self.selector.last_raw, selected_id)
            except Exception as exc:
                rospy.logwarn('Qwen frontier 선택 실패 → 규칙 기반 fallback: %s', exc)
        selected = next(
            (item for item in candidates if item['id'] == selected_id), candidates[0])

        self.goal_cell = selected['cell']
        self.path = selected['path']
        self.recent_goal_ids.append({
            'id': selected['id'], 'cell': list(selected['cell'])})
        self.recent_goal_ids = self.recent_goal_ids[-10:]
        self.state = 'NAVIGATING'
        self.last_plan_time = time.monotonic()
        self.last_progress_pose = pose
        self.last_progress_time = self.last_plan_time
        self.publish_goal_and_path(grid)
        self.publish_status(
            'frontier %d 선택 (%.2fm)' % (selected['id'], selected['distance_m']))

    def publish_goal_and_path(self, grid):
        now = rospy.Time.now()
        if self.goal_cell is not None:
            gx, gy = grid.cell_to_world(self.goal_cell)
            goal = PoseStamped()
            goal.header.stamp = now
            goal.header.frame_id = self.map_frame
            goal.pose.position.x = gx
            goal.pose.position.y = gy
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)

        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = self.map_frame
        for cell in self.path:
            point = PoseStamped()
            point.header = path_msg.header
            point.pose.position.x, point.pose.position.y = grid.cell_to_world(cell)
            point.pose.orientation.w = 1.0
            path_msg.poses.append(point)
        self.path_pub.publish(path_msg)

    def replan_to_goal(self, grid, pose):
        start = grid.world_to_cell(pose[0], pose[1])
        blocked = grid.inflated_obstacles(self.robot_radius)
        path = grid.astar(start, self.goal_cell, blocked)
        if not path:
            self.fail_current_goal('현재 frontier 경로 소실')
            return False
        self.path = path
        self.last_plan_time = time.monotonic()
        self.publish_goal_and_path(grid)
        return True

    def control_tick(self, _event):
        now = time.monotonic()
        with self.lock:
            running = self.running
            grid = self.grid
            pose = self.pose
            front_distance = self.front_distance
            exit_detected = self.exit_detected
            map_age = now - self.map_time
            pose_age = now - self.pose_time
            scan_age = now - self.scan_time

            if not running:
                self.publish_stop()
                return
            if exit_detected:
                self.running = False
                self.state = 'EXIT_FOUND'
                self.publish_stop(repeat=3)
                self.publish_status('카메라 출구 감지')
                return
            if grid is None or pose is None or max(map_age, pose_age, scan_age) > self.data_timeout:
                self.publish_stop()
                self.publish_status('map/pose/scan 대기 또는 시간 초과')
                return
            if front_distance is None or not math.isfinite(front_distance):
                self.publish_stop()
                self.publish_status('전방 LiDAR 유효값 없음')
                return

            if self.state == 'SELECTING' or self.goal_cell is None or not self.path:
                self.select_goal(grid, pose)
                return

            gx, gy = grid.cell_to_world(self.goal_cell)
            if math.hypot(gx - pose[0], gy - pose[1]) <= self.goal_tolerance:
                self.goal_cell = None
                self.path = []
                self.state = 'SELECTING'
                self.publish_stop()
                self.publish_status('frontier 도착, 다음 목표 탐색')
                return

            if now - self.last_plan_time >= self.replan_period:
                if not self.replan_to_goal(grid, pose):
                    return

            target = choose_lookahead(
                self.path, grid, pose[0], pose[1], self.lookahead)
            if target is None:
                self.fail_current_goal('추종할 경로점 없음')
                return
            target_yaw = math.atan2(target[1] - pose[1], target[0] - pose[0])
            error = wrap_angle(target_yaw - pose[2])

            # 전방 장애물은 전진하려는 경우에만 목표 실패로 처리한다.
            # 회전이 필요한 막다른 길에서는 제자리 회전을 허용해야 탈출할 수 있다.
            if abs(error) < self.rotate_threshold and front_distance <= self.stop_distance:
                self.publish_stop()
                if self.blocked_since is None:
                    self.blocked_since = now
                elif now - self.blocked_since >= self.blocked_timeout:
                    self.fail_current_goal('전방 장애물로 frontier 포기')
                return
            self.blocked_since = None

            if self.last_progress_pose is None or math.hypot(
                    pose[0] - self.last_progress_pose[0],
                    pose[1] - self.last_progress_pose[1]) >= 0.10 or abs(wrap_angle(
                        pose[2] - self.last_progress_pose[2])) >= 0.12:
                self.last_progress_pose = pose
                self.last_progress_time = now
            elif now - self.last_progress_time >= self.stuck_timeout:
                self.fail_current_goal('위치 변화 없음, frontier 재선택')
                return

            if abs(error) >= self.rotate_threshold:
                yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate,
                                                       self.turn_gain * error))
                self.velocity_pub.publish(Velocity(x=0.0, y=0.0, yaw_rate=yaw_rate))
                return

            speed = self.forward_speed
            if front_distance < self.slow_distance:
                scale = max(0.35, (front_distance - self.stop_distance) /
                            max(0.01, self.slow_distance - self.stop_distance))
                speed *= scale
            yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate,
                                                   self.heading_gain * error))
            self.velocity_pub.publish(Velocity(x=speed, y=0.0, yaw_rate=yaw_rate))

    def shutdown(self):
        self.running = False
        self.publish_stop(repeat=3)


if __name__ == '__main__':
    MazeAutonomyNode()
    rospy.spin()

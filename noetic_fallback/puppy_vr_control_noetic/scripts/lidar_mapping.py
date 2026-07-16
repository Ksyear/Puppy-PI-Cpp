#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_mapping (ROS1 Noetic)
===========================
LiDAR(/scan)로 2D 점유격자 지도를 만들고,
  1) ROS 토픽 /map (nav_msgs/OccupancyGrid, latch) 으로 발행하고    -- rviz/rosbridge 용
  2) 지도 이미지를 UDP 5008 로 전송한다 (영상과 같은 청크 프로토콜) -- 대시보드/VR 용

알고리즘: ROS2판 lidar_mapping_cpp(mapping_core.hpp)의 파이썬 이식.
  - 점유격자를 로그오즈로 유지 (광선 경로 비움, 끝점 점유)
  - 오도메트리 없이 언덕오르기 스캔매칭으로 자세(x, y, theta) 추정
  - 5cm/0.05rad 이상 움직였을 때만 지도에 통합
(시뮬레이션 검증은 C++판에서 완료: 60스텝 주행 후 위치 오차 5cm — 09번 문서)

사용:
  roslaunch puppy_vr_control_noetic lidar_mapping.launch
  또는 run_vr.sh use_mapping:=true  (VR 조종과 함께)
지도 저장:
  rosservice call /lidar_mapping/save_map     # ~/maps/ros1_map.pgm + .yaml
"""

import math
import os
import socket
import struct
import threading
import time

import numpy as np
import rospy
import tf

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerResponse

try:
    import cv2
except ImportError:
    cv2 = None   # 로봇에는 있음. 없으면 광선 추적이 파이썬 루프(느림)로 대체


class Mapper(object):
    """순수 지도작성 로직 (rospy 비의존)."""
    HIT, MISS, LO_MIN, LO_MAX = 24, -6, -120, 120

    def __init__(self, size_m=10.0, resolution=0.05):
        self.res = resolution
        self.n = int(size_m / resolution)          # 한 변 셀 수
        self.origin = -size_m / 2.0                # 지도 중앙이 시작점 (0,0)
        self.lo = np.zeros((self.n, self.n), dtype=np.int16)    # [cy, cx]
        self.known = np.zeros((self.n, self.n), dtype=bool)
        self.pose = np.array([0.0, 0.0, 0.0])      # x, y, theta
        self.last_integrated = self.pose.copy()
        self.initialized = False

    def world_to_cell(self, xy):
        """(N,2) 월드좌표 → (N,2) 셀 인덱스 (경계 밖 포함, 정수)."""
        return np.floor((xy - self.origin) / self.res).astype(np.int32)

    def score(self, pose, pts):
        """스캔 끝점들이 기존 점유 셀에 얼마나 얹히는지 (클수록 좋음)."""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world = pts @ np.array([[c, s], [-s, c]]) + pose[:2]
        cells = self.world_to_cell(world)
        ok = (cells[:, 0] >= 0) & (cells[:, 0] < self.n) & \
             (cells[:, 1] >= 0) & (cells[:, 1] < self.n)
        cells = cells[ok]
        return int(self.lo[cells[:, 1], cells[:, 0]].sum())

    def match(self, pts, lin_step=0.05, ang_step=0.03, halvings=4, max_iters=200):
        """언덕오르기 스캔매칭: (±x, ±y, ±theta) 이웃 시도, 개선 없으면 보폭 절반."""
        best = self.pose.copy()
        best_score = self.score(best, pts)
        lin, ang = lin_step, ang_step
        rounds = 0
        level = 0
        while level <= halvings and rounds < max_iters:   # C++ 판과 동일: 라운드 수 기준
            rounds += 1
            improved = False
            for d in ((lin, 0, 0), (-lin, 0, 0), (0, lin, 0),
                      (0, -lin, 0), (0, 0, ang), (0, 0, -ang)):
                cand = best + np.array(d)
                sc = self.score(cand, pts)
                if sc > best_score:
                    best_score, best, improved = sc, cand, True
            if not improved:
                lin, ang = lin / 2.0, ang / 2.0
                level += 1
        return best

    def integrate(self, pts):
        """현재 자세에서 스캔 반영: 광선 경로 = 비움(MISS), 끝점 = 점유(HIT)."""
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        world = pts @ np.array([[c, s], [-s, c]]) + self.pose[:2]
        ends = self.world_to_cell(world)
        p0 = self.world_to_cell(self.pose[None, :2])[0]

        # 지도 밖으로 나가는 광선은 통째로 제외 (C++ 판과 동일 — 부분 반영 왜곡 방지)
        ok = (ends[:, 0] >= 0) & (ends[:, 0] < self.n) & \
             (ends[:, 1] >= 0) & (ends[:, 1] < self.n)
        ends = ends[ok]
        if len(ends) == 0 or not (0 <= p0[0] < self.n and 0 <= p0[1] < self.n):
            return

        # MISS 는 광선이 지나갈 때마다 누적 (스캔당 1회 아님) —
        # 잘못 통합된 옛 벽이 관통 광선들에 의해 빠르게 지워져 지도가 자기수정된다
        free_cnt = np.zeros((self.n, self.n), dtype=np.int32)
        if cv2 is not None:
            tmp = np.zeros((self.n, self.n), dtype=np.uint8)
            for ex, ey in ends:
                tmp[:] = 0
                cv2.line(tmp, (int(p0[0]), int(p0[1])), (int(ex), int(ey)), 1, 1)
                free_cnt += tmp
        else:   # cv2 없을 때의 느린 대체 경로 (오프라인 테스트용)
            for ex, ey in ends:
                x0, y0, x1, y1 = int(p0[0]), int(p0[1]), int(ex), int(ey)
                dx, dy = abs(x1 - x0), abs(y1 - y0)
                sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
                err = dx - dy
                while True:
                    free_cnt[y0, x0] += 1
                    if x0 == x1 and y0 == y1:
                        break
                    e2 = 2 * err
                    if e2 > -dy:
                        err -= dy
                        x0 += sx
                    if e2 < dx:
                        err += dx
                        y0 += sy

        # 끝점 셀은 비움 대상에서 제외하고 광선당 HIT 누적
        free_cnt[ends[:, 1], ends[:, 0]] = 0
        lo32 = self.lo.astype(np.int32) + self.MISS * free_cnt
        np.add.at(lo32, (ends[:, 1], ends[:, 0]), self.HIT)
        self.lo = np.clip(lo32, self.LO_MIN, self.LO_MAX).astype(np.int16)
        self.known |= free_cnt > 0
        self.known[ends[:, 1], ends[:, 0]] = True
        self.last_integrated = self.pose.copy()

    def process(self, pts, min_travel=0.05, min_heading=0.05):
        """스캔 1회 처리: 자세 추정 + 필요 시 지도 통합. 현재 자세 반환."""
        if not self.initialized:
            self.integrate(pts)
            self.initialized = True
            return self.pose
        self.pose = self.match(pts)
        d = self.pose - self.last_integrated
        turned = abs(math.atan2(math.sin(d[2]), math.cos(d[2])))
        if math.hypot(d[0], d[1]) >= min_travel or turned >= min_heading:
            self.integrate(pts)
        return self.pose

    def occupancy(self):
        """ROS OccupancyGrid 규약의 int8 배열: -1 미탐사 / 0~100."""
        p = (100.0 / (1.0 + np.exp(-self.lo * 0.05))).round().astype(np.int8)
        p[~self.known] = -1
        return p

    def render_bgr(self, view_m=6.0, scale=3):
        """대시보드 전송용 이미지 (로봇 중심 시점).
        로봇을 **항상 화면 정중앙에 고정**하고 주변 view_m×view_m 영역만 잘라 보낸다.
        로봇이 움직이면 지도가 로봇 밑에서 스크롤되므로 로봇이 화면 밖으로 사라지지 않는다.
        흰=빈공간, 검=벽, 회색=미탐사, 로봇=빨강 화살표(중앙)."""
        if cv2 is None:
            return None
        # 1) 전체 점유격자 → 회색조 (아직 상하반전 전: [cy, cx])
        occ = self.occupancy()
        full = np.full((self.n, self.n), 205, dtype=np.uint8)   # 미탐사 회색
        full[(occ >= 0) & (occ <= 25)] = 254
        full[occ >= 65] = 0

        # 2) 로봇 셀을 중심으로 view_cells×view_cells 창을 잘라냄 (지도 밖은 회색 패딩).
        #    홀수로 맞춰 로봇이 정확히 중앙 셀에 오게 한다.
        view_cells = max(21, int(view_m / self.res)) | 1
        cc = view_cells // 2
        irx = int((self.pose[0] - self.origin) / self.res)
        iry = int((self.pose[1] - self.origin) / self.res)
        crop = np.full((view_cells, view_cells), 205, dtype=np.uint8)
        sr0, sc0 = iry - cc, irx - cc          # 창 좌상단이 가리키는 전체격자 인덱스
        r0, r1 = max(0, sr0), min(self.n, sr0 + view_cells)
        c0, c1 = max(0, sc0), min(self.n, sc0 + view_cells)
        if r1 > r0 and c1 > c0:
            crop[r0 - sr0:r1 - sr0, c0 - sc0:c1 - sc0] = full[r0:r1, c0:c1]

        # 3) 이미지 y축은 아래로 → 상하반전 후 확대(격자 또렷하게 nearest)
        crop = np.flipud(crop)
        bgr = cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)
        out = view_cells * scale
        bgr = cv2.resize(bgr, (out, out), interpolation=cv2.INTER_NEAREST)

        # 4) 로봇은 항상 정중앙 — 방향 화살표(로봇 heading) 표시
        c = out // 2
        L = max(10, out // 12)
        hx = int(c + L * math.cos(-self.pose[2]))
        hy = int(c + L * math.sin(-self.pose[2]))
        cv2.circle(bgr, (c, c), max(4, L // 3), (0, 0, 255), -1)
        cv2.line(bgr, (c, c), (hx, hy), (0, 0, 255), 2)
        return bgr


class LidarMappingNode(object):

    def __init__(self):
        rospy.init_node('lidar_mapping')

        scan_topic = rospy.get_param('~scan_topic', '')   # '' = LaserScan 토픽 자동 탐지
        size_m = rospy.get_param('~map_size_m', 12.0)
        resolution = rospy.get_param('~resolution', 0.05)
        self.max_range = rospy.get_param('~max_range', 8.0)
        self.min_range = rospy.get_param('~min_range', 0.15)
        # 빔 부분샘플: 반드시 1 유지 권장 — 시뮬레이션에서 2로 줄이면
        # 스캔매칭 증거가 부족해져 추적이 발산함 (A/B 실험으로 확인)
        self.beam_step = rospy.get_param('~beam_step', 1)
        self.laser_yaw = rospy.get_param('~laser_yaw', 0.0)   # 장착 방향 보정 (거꾸로면 3.14159)
        self.bind_port = rospy.get_param('~bind_port', 5008)
        self.client_timeout_sec = rospy.get_param('~client_timeout_sec', 5.0)
        self.map_send_period = rospy.get_param('~map_send_period', 1.0)
        self.chunk_size = rospy.get_param('~chunk_size', 1400)
        self.save_dir = os.path.expanduser(rospy.get_param('~map_save_path', '~/maps'))
        self.map_name = rospy.get_param('~map_name', 'ros1_map')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.pose_topic = rospy.get_param('~pose_topic', '/lidar_mapping/pose')
        self.publish_tf = rospy.get_param('~publish_tf', True)
        # 로봇 중심 지도 뷰에서 로봇 주변으로 보여줄 범위(m). 작을수록 확대됨.
        self.view_m = rospy.get_param('~view_m', 6.0)

        self.mapper = Mapper(size_m, resolution)
        self.lock = threading.Lock()

        # UDP (hello 자동 발견 — 영상/상태와 동일 방식)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.bind_port))
        self.sock.settimeout(0.2)
        self.client_addr = None
        self.last_hello = 0.0
        self.frame_counter = 0
        self.running = True
        threading.Thread(target=self.hello_loop, daemon=True).start()

        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Service('~save_map', Trigger, self.save_map)
        rospy.Timer(rospy.Duration(self.map_send_period), self.publish_and_send)

        if scan_topic:
            rospy.Subscriber(scan_topic, LaserScan, self.scan_cb, queue_size=1)
            rospy.loginfo('지도작성 시작: %s (%.0fx%.0fm @ %.2fm)',
                          scan_topic, size_m, size_m, resolution)
        else:
            threading.Thread(target=self.autodetect_scan, daemon=True).start()

        rospy.on_shutdown(self.shutdown)

    def autodetect_scan(self):
        while self.running and not rospy.is_shutdown():
            try:
                topics = rospy.get_published_topics()
            except Exception:
                time.sleep(2)
                continue
            scans = [t for t, ty in topics if ty == 'sensor_msgs/LaserScan']
            if scans:
                scans.sort(key=len)
                rospy.Subscriber(scans[0], LaserScan, self.scan_cb, queue_size=1)
                rospy.loginfo('LiDAR 토픽 자동 감지: %s → 지도작성 시작', scans[0])
                return
            rospy.logwarn_throttle(
                15, 'LaserScan 토픽 없음 — LiDAR 노드 실행 확인 (rostopic list | grep -i scan)')
            time.sleep(2)

    def scan_cb(self, msg):
        # LaserScan → base 기준 점 목록 (부분샘플 + 유효거리 필터)
        r = np.asarray(msg.ranges[::self.beam_step], dtype=np.float64)
        a = msg.angle_min + msg.angle_increment * self.beam_step * \
            np.arange(len(r)) + self.laser_yaw
        ok = np.isfinite(r) & (r >= self.min_range) & (r <= self.max_range)
        if ok.sum() < 30:
            return
        pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)
        with self.lock:
            pose = self.mapper.process(pts).copy()
        self.publish_pose(pose, msg.header.stamp)

    def publish_pose(self, pose, stamp):
        """스캔매칭 자세를 자율주행 노드가 사용할 수 있도록 토픽과 TF로 공개."""
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, float(pose[2]))

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(pose[0])
        msg.pose.position.y = float(pose[1])
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        self.pose_pub.publish(msg)

        if self.publish_tf:
            self.tf_broadcaster.sendTransform(
                (float(pose[0]), float(pose[1]), 0.0), quat, stamp,
                self.base_frame, self.map_frame)

    def hello_loop(self):
        while self.running and not rospy.is_shutdown():
            try:
                _, addr = self.sock.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break
            if addr != self.client_addr:
                rospy.loginfo('지도 수신 클라이언트: %s:%d', addr[0], addr[1])
            self.client_addr = addr
            self.last_hello = time.monotonic()

    def publish_and_send(self, _event):
        with self.lock:
            if not self.mapper.initialized:
                return
            occ = self.mapper.occupancy()
            bgr = self.mapper.render_bgr(view_m=self.view_m)

        # 1) ROS 토픽 (rviz / rosbridge 용)
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = self.mapper.res
        grid.info.width = grid.info.height = self.mapper.n
        grid.info.origin.position.x = self.mapper.origin
        grid.info.origin.position.y = self.mapper.origin
        grid.info.origin.orientation.w = 1.0
        grid.data = occ.flatten().tolist()
        self.map_pub.publish(grid)

        # 2) UDP 전송 (대시보드/VR 용, 영상과 같은 청크 프로토콜)
        if bgr is None or self.client_addr is None:
            return
        if time.monotonic() - self.last_hello > self.client_timeout_sec:
            return
        ok, buf = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return
        jpeg = buf.tobytes()
        total = len(jpeg)
        count = (total + self.chunk_size - 1) // self.chunk_size
        fid = self.frame_counter & 0xFFFFFFFF
        self.frame_counter += 1
        for i in range(count):
            header = struct.pack('!IHHI', fid, i, count, total)
            try:
                self.sock.sendto(header + jpeg[i * self.chunk_size:(i + 1) * self.chunk_size],
                                 self.client_addr)
            except OSError:
                return

    def save_map(self, _req):
        """map_server 형식(pgm+yaml)으로 저장 — Nav2/map_server 에서 재사용 가능."""
        with self.lock:
            occ = self.mapper.occupancy()
        os.makedirs(self.save_dir, exist_ok=True)
        pgm = os.path.join(self.save_dir, self.map_name + '.pgm')
        img = np.full(occ.shape, 205, dtype=np.uint8)
        img[(occ >= 0) & (occ <= 25)] = 254
        img[occ >= 65] = 0
        img = np.flipud(img)
        with open(pgm, 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (occ.shape[1], occ.shape[0]))
            f.write(img.tobytes())
        with open(os.path.join(self.save_dir, self.map_name + '.yaml'), 'w') as f:
            f.write('image: %s.pgm\nresolution: %f\norigin: [%f, %f, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
                    % (self.map_name, self.mapper.res, self.mapper.origin, self.mapper.origin))
        rospy.loginfo('지도 저장: %s', pgm)
        return TriggerResponse(success=True, message=pgm)

    def shutdown(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass


if __name__ == '__main__':
    LidarMappingNode()
    rospy.spin()

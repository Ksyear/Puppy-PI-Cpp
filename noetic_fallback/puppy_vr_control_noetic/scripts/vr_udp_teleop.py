#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vr_udp_teleop (ROS1 Noetic 비상용)
==================================
ROS2판(puppy_vr_control_py/vr_udp_teleop.py)의 rospy 이식.
UDP 프로토콜·파라미터·안전장치(1초 무신호 정지, ESTOP/RESUME, 속도 클램프) 동일.

ROS1 스택에서도 토픽 이름이 같음을 원본(ros1 브랜치 remote_control_joystick.py)으로
확인함: /puppy_control/velocity/autogait (puppy_control/Velocity)
"""

import math
import socket
import threading
import time

import rospy

from puppy_control.msg import Velocity
from std_srvs.srv import Empty


def parse_packet(text):
    """'X:12.3,Z:-5.0' → (x, z) 또는 None. ROS2판과 동일 규칙."""
    values = {}
    for part in text.strip().split(','):
        if ':' not in part:
            continue
        key, _, val = part.partition(':')
        key = key.strip().upper()
        try:
            values[key] = float(val.strip())
        except ValueError:
            return None
    if 'X' not in values or 'Z' not in values:
        return None
    return values['X'], values['Z']


class VrUdpTeleop(object):

    def __init__(self):
        rospy.init_node('vr_udp_teleop')

        self.port = rospy.get_param('~port', 5005)
        self.deadzone_deg = rospy.get_param('~deadzone_deg', 5.0)
        self.max_angle_deg = rospy.get_param('~max_angle_deg', 45.0)
        self.recv_timeout_sec = rospy.get_param('~recv_timeout_sec', 1.0)
        self.max_speed_x = min(rospy.get_param('~max_speed_x', 15.0), 35.0)
        self.max_yaw_rate_deg = min(rospy.get_param('~max_yaw_rate_deg', 20.0), 51.0)
        self.invert_forward = rospy.get_param('~invert_forward', False)
        self.invert_turn = rospy.get_param('~invert_turn', True)
        self.velocity_topic = rospy.get_param(
            '~velocity_topic', '/puppy_control/velocity/autogait')
        self.publish_rate = rospy.get_param('~publish_rate', 20.0)
        self.speed_step = rospy.get_param('~speed_step', 1.0)
        self.yaw_step_deg = rospy.get_param('~yaw_step_deg', 2.0)
        self.heartbeat_sec = rospy.get_param('~heartbeat_sec', 0.5)
        # 제자리 회전 시 로봇이 뒤로 밀리면 1.0~2.5 정도로 (cm/s, 회전 중 전진 보정)
        self.turn_forward_bias = rospy.get_param('~turn_forward_bias', 0.0)
        self.debug = rospy.get_param('~debug', False)

        self.velocity_pub = rospy.Publisher(self.velocity_topic, Velocity, queue_size=1)

        self.lock = threading.Lock()
        self.cmd_x_angle = 0.0
        self.cmd_z_angle = 0.0
        self.have_cmd = False
        self.last_rx = 0.0
        self.estop = False

        self.moving = False
        self.stop_repeat = 0
        self.last_vx = 0.0
        self.last_vyaw = 0.0
        self.last_pub = 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.settimeout(0.1)

        self.running = True
        threading.Thread(target=self.rx_loop, daemon=True).start()

        # 시작 시 서기 자세 (go_home) — 엎드린 채 시작하지 않도록
        if rospy.get_param('~stand_on_start', True):
            threading.Thread(target=self.stand_up, daemon=True).start()

        rospy.on_shutdown(self.shutdown)
        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 1.0)), self.control_tick)

        rospy.loginfo('VR UDP 수신 대기(ROS1): 0.0.0.0:%d -> %s',
                      self.port, self.velocity_topic)

    def stand_up(self):
        """다리 펴고(서기) 시작: /puppy_control/go_home 1회 호출."""
        try:
            rospy.wait_for_service('/puppy_control/go_home', timeout=15)
            rospy.ServiceProxy('/puppy_control/go_home', Empty)()
            rospy.loginfo('시작 자세: go_home(서기) 호출 완료')
        except Exception as e:
            rospy.logwarn('go_home 호출 실패 — 수동 실행: rosservice call /puppy_control/go_home (%s)', e)

    def check_estop(self, text):
        if 'ESTOP' in text:
            if not self.estop:
                rospy.logerr('[ESTOP] 긴급 정지 수신 — RESUME 전까지 이동 명령 무시')
            self.estop = True
            return True
        if 'RESUME' in text:
            if self.estop:
                rospy.logwarn('[RESUME] 긴급 정지 해제')
            self.estop = False
            return True
        return False

    def rx_loop(self):
        while self.running and not rospy.is_shutdown():
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            text = data.decode('utf-8', errors='replace')
            latest = None if self.check_estop(text) else parse_packet(text)
            # 밀린 패킷은 비우고 최신 값만 사용 (지연 방지)
            self.sock.setblocking(False)
            try:
                while True:
                    try:
                        data, addr = self.sock.recvfrom(1024)
                    except (BlockingIOError, OSError):
                        break
                    text = data.decode('utf-8', errors='replace')
                    if self.check_estop(text):
                        continue
                    parsed = parse_packet(text)
                    if parsed is not None:
                        latest = parsed
            finally:
                self.sock.setblocking(True)
                self.sock.settimeout(0.1)

            if latest is None:
                continue
            with self.lock:
                self.cmd_x_angle, self.cmd_z_angle = latest
                self.last_rx = time.monotonic()
                self.have_cmd = True
            if self.debug:
                rospy.loginfo_throttle(0.5, '[수신] X=%.1f Z=%.1f' % latest)

    def normalize(self, angle):
        if abs(angle) < self.deadzone_deg:
            return 0.0
        return max(-1.0, min(1.0, angle / self.max_angle_deg))

    def control_tick(self, _event):
        with self.lock:
            x_angle = self.cmd_x_angle
            z_angle = self.cmd_z_angle
            have_cmd = self.have_cmd
            last_rx = self.last_rx

        now = time.monotonic()

        if self.estop:
            if self.moving:
                self.publish_stop()
                self.moving = False
                self.stop_repeat = 3
            elif self.stop_repeat > 0:
                self.publish_stop()
                self.stop_repeat -= 1
            return

        if not have_cmd or now - last_rx > self.recv_timeout_sec:
            if self.moving:
                self.publish_stop()
                self.moving = False
                rospy.logwarn('[무신호 %.1fs] 안전 정지', self.recv_timeout_sec)
            elif self.stop_repeat > 0:
                self.publish_stop()
                self.stop_repeat -= 1
            return

        forward = self.normalize(z_angle) * (-1.0 if self.invert_forward else 1.0)
        turn = self.normalize(x_angle) * (-1.0 if self.invert_turn else 1.0)

        yaw_step = math.radians(self.yaw_step_deg)
        max_yaw = math.radians(self.max_yaw_rate_deg)
        vx = round(forward * self.max_speed_x / self.speed_step) * self.speed_step
        vyaw = round(turn * max_yaw / yaw_step) * yaw_step
        vx = max(-self.max_speed_x, min(self.max_speed_x, vx))
        vyaw = max(-max_yaw, min(max_yaw, vyaw))

        if vx == 0.0 and vyaw == 0.0:
            if self.moving:
                self.publish_stop()
                self.moving = False
            return

        # 제자리 회전 보정: 회전만 할 때 뒤로 밀리는 것을 전진 성분으로 상쇄
        if vx == 0.0 and vyaw != 0.0 and self.turn_forward_bias != 0.0:
            vx = float(self.turn_forward_bias)

        if vx != self.last_vx or vyaw != self.last_vyaw or now - self.last_pub >= self.heartbeat_sec:
            self.velocity_pub.publish(Velocity(x=vx, y=0.0, yaw_rate=vyaw))
            self.last_vx = vx
            self.last_vyaw = vyaw
            self.last_pub = now
            self.moving = True
            self.stop_repeat = 3
            if self.debug:
                rospy.loginfo_throttle(0.5, '[발행] x=%+.1f yaw=%+.2f' % (vx, vyaw))

    def publish_stop(self):
        self.velocity_pub.publish(Velocity(x=0.0, y=0.0, yaw_rate=0.0))
        self.last_vx = 0.0
        self.last_vyaw = 0.0
        self.last_pub = time.monotonic()

    def shutdown(self):
        self.running = False
        try:
            self.publish_stop()
            self.sock.close()
        except Exception:
            pass


if __name__ == '__main__':
    VrUdpTeleop()
    rospy.spin()

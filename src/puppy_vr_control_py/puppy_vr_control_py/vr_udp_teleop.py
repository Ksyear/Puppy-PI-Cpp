#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vr_udp_teleop (파이썬판)
========================
Meta Quest(Unity)의 UDP_Joystick_Sender.cs 가 보내는 조이스틱 기울기 각도
("X:12.3,Z:-5.0" 형식, 기본 20Hz)를 UDP로 수신해서
puppy_control_msgs/Velocity 로 변환해 /puppy_control/velocity/autogait 로 발행한다.

C++판(src/puppy_vr_control/src/vr_udp_teleop.cpp)과 프로토콜·파라미터·동작이 동일하다.
파이썬 스택으로 로봇을 돌릴 때는 이 노드를 사용하면 된다.

변환 규칙 (PuppyPi_MR_Controller 원본과 동일):
  - 데드존: |각도| < deadzone_deg 이면 0
  - 정규화: 각도 / max_angle_deg 를 -1.0 ~ 1.0 으로 클램프
  - Z(앞뒤 기울기) → 전진/후진,  X(좌우 기울기) → 회전
  - recv_timeout_sec 동안 패킷이 없으면 안전 정지

PuppyPi 단위계 (puppy.py 참고):
  - Velocity.x  : cm/s 스케일, |x| <= 35 (넘으면 puppy_control이 무시)
  - Velocity.y  : 반드시 0
  - Velocity.yaw_rate : rad/s, |yaw| <= radians(51)
"""

import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node

from puppy_control_msgs.msg import Velocity
from std_srvs.srv import Empty


def parse_packet(text):
    """'X:12.3,Z:-5.0' → (x, z) 또는 None (형식 오류 시). 원본 수신기와 동일 규칙."""
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


class VrUdpTeleop(Node):

    def __init__(self):
        super().__init__('vr_udp_teleop')

        # ── 파라미터 (C++판과 동일 기본값) ──
        self.port = self.declare_parameter('port', 5005).value
        self.deadzone_deg = self.declare_parameter('deadzone_deg', 5.0).value
        self.max_angle_deg = self.declare_parameter('max_angle_deg', 45.0).value
        self.recv_timeout_sec = self.declare_parameter('recv_timeout_sec', 1.0).value

        # 최대 속도: 공식 조이스틱 노드 기본값(VelocityX=15, yaw ±20°)과 동일
        self.max_speed_x = self.declare_parameter('max_speed_x', 15.0).value      # cm/s
        self.max_yaw_rate_deg = self.declare_parameter('max_yaw_rate_deg', 20.0).value

        # 실기기에서 방향이 반대로 움직이면 이 두 개를 뒤집으면 된다
        self.invert_forward = self.declare_parameter('invert_forward', False).value
        self.invert_turn = self.declare_parameter('invert_turn', True).value

        self.velocity_topic = self.declare_parameter(
            'velocity_topic', '/puppy_control/velocity/autogait').value
        self.publish_rate = self.declare_parameter('publish_rate', 20.0).value

        # 같은 값 반복 발행으로 gait_config가 매번 재계산되는 것을 막는 양자화 폭
        self.speed_step = self.declare_parameter('speed_step', 1.0).value        # cm/s
        self.yaw_step_deg = self.declare_parameter('yaw_step_deg', 2.0).value    # deg/s
        self.heartbeat_sec = self.declare_parameter('heartbeat_sec', 0.5).value

        self.debug = self.declare_parameter('debug', False).value

        # puppy_control 이 조용히 무시하는 범위를 넘지 않도록 상한 강제
        if self.max_speed_x > 35.0:
            self.get_logger().warn(f'max_speed_x {self.max_speed_x} > 35 → 35로 제한')
            self.max_speed_x = 35.0
        if self.max_yaw_rate_deg > 51.0:
            self.get_logger().warn(f'max_yaw_rate_deg {self.max_yaw_rate_deg} > 51 → 51로 제한')
            self.max_yaw_rate_deg = 51.0

        self.velocity_pub = self.create_publisher(Velocity, self.velocity_topic, 1)

        # ── 공유 상태 (수신 스레드 ↔ 제어 타이머) ──
        self.lock = threading.Lock()
        self.cmd_x_angle = 0.0
        self.cmd_z_angle = 0.0
        self.have_cmd = False
        self.last_rx = 0.0

        self.moving = False
        self.estop = False
        self.stop_repeat = 0
        self.last_vx = 0.0
        self.last_vyaw = 0.0
        self.last_pub = 0.0
        self.last_debug_print = 0.0

        # ── UDP 소켓 + 수신 스레드 ──
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.settimeout(0.1)  # 0.1초마다 종료 플래그 확인

        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self.control_tick)

        # 시작 시 서기 자세 (go_home) — 엎드린 채 시작하지 않도록
        if self.declare_parameter('stand_on_start', True).value:
            self.go_home_client = self.create_client(Empty, '/puppy_control/go_home')
            self.stand_tries = 0
            self.stand_timer = self.create_timer(1.0, self.try_stand)

        self.get_logger().info(
            f'VR UDP 조이스틱 수신 대기: 0.0.0.0:{self.port} '
            f'(데드존 ±{self.deadzone_deg}°, 최대각 {self.max_angle_deg}°, '
            f'무신호정지 {self.recv_timeout_sec}s)')
        self.get_logger().info(
            f'발행 토픽: {self.velocity_topic} '
            f'(최대 x={self.max_speed_x} cm/s, yaw={self.max_yaw_rate_deg} deg/s)')

    def try_stand(self):
        """go_home 서비스가 준비되면 1회 호출해 다리 펴고(서기) 시작."""
        self.stand_tries += 1
        if self.go_home_client.service_is_ready():
            self.go_home_client.call_async(Empty.Request())
            self.get_logger().info('시작 자세: go_home(서기) 호출')
            self.stand_timer.cancel()
        elif self.stand_tries > 15:
            self.get_logger().warn('go_home 서비스 없음 — 서기 자세 생략 (puppy_control 확인)')
            self.stand_timer.cancel()

    def check_estop(self, text):
        """긴급정지 프로토콜: 'ESTOP' → 즉시 정지+명령 무시, 'RESUME' → 해제. 명령이면 True."""
        if 'ESTOP' in text:
            if not self.estop:
                self.get_logger().error('[ESTOP] 긴급 정지 수신 — RESUME 전까지 모든 이동 명령 무시')
            self.estop = True
            return True
        if 'RESUME' in text:
            if self.estop:
                self.get_logger().warn('[RESUME] 긴급 정지 해제')
            self.estop = False
            return True
        return False

    def rx_loop(self):
        """수신 스레드: 밀린 패킷을 모두 비우고 가장 최근의 유효한 값만 저장 (지연 방지)."""
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            text = data.decode('utf-8', errors='replace')
            latest = None if self.check_estop(text) else parse_packet(text)
            # 큐에 쌓인 나머지 패킷도 논블로킹으로 비운다
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
                now = time.monotonic()
                if now - self.last_debug_print >= 0.5:
                    self.last_debug_print = now
                    self.get_logger().info(
                        f'[수신] {addr[0]}  X={latest[0]:6.1f} Z={latest[1]:6.1f}')

    def normalize(self, angle):
        """데드존 + 정규화 (원본 to_velocity의 norm과 동일)."""
        if abs(angle) < self.deadzone_deg:
            return 0.0
        return max(-1.0, min(1.0, angle / self.max_angle_deg))

    def control_tick(self):
        """주기 제어 루프: 최신 명령 → Velocity 발행 + 무신호 안전 정지."""
        with self.lock:
            x_angle = self.cmd_x_angle
            z_angle = self.cmd_z_angle
            have_cmd = self.have_cmd
            last_rx = self.last_rx

        now = time.monotonic()

        # ── 긴급 정지: 정지 명령만 발행, 이동 명령 무시 ──
        if self.estop:
            if self.moving:
                self.publish_stop()
                self.moving = False
                self.stop_repeat = 3
            elif self.stop_repeat > 0:
                self.publish_stop()
                self.stop_repeat -= 1
            return

        # ── 무신호 안전 정지 ──
        if not have_cmd or now - last_rx > self.recv_timeout_sec:
            if self.moving:
                self.publish_stop()
                self.moving = False
                self.get_logger().warn(f'[무신호 {self.recv_timeout_sec}s] 안전 정지')
            elif self.stop_repeat > 0:
                self.publish_stop()  # 정지 명령 유실 대비 몇 번 더 발행
                self.stop_repeat -= 1
            return

        # ── 각도 → 속도 변환 ──
        forward = self.normalize(z_angle) * (-1.0 if self.invert_forward else 1.0)
        turn = self.normalize(x_angle) * (-1.0 if self.invert_turn else 1.0)

        # 양자화: 스틱 미세 떨림으로 gait_config가 계속 재계산되는 것 방지
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

        # 값이 바뀌었거나 하트비트 주기가 지났을 때만 발행
        if vx != self.last_vx or vyaw != self.last_vyaw or now - self.last_pub >= self.heartbeat_sec:
            self.velocity_pub.publish(Velocity(x=float(vx), y=0.0, yaw_rate=float(vyaw)))
            self.last_vx = vx
            self.last_vyaw = vyaw
            self.last_pub = now
            self.moving = True
            self.stop_repeat = 3

            if self.debug:
                self.get_logger().info(f'[발행] x={vx:+.1f} cm/s  yaw={vyaw:+.2f} rad/s')

    def publish_stop(self):
        # x=0, y=0, yaw_rate=0 → puppy_control 이 move_stop 수행
        self.velocity_pub.publish(Velocity(x=0.0, y=0.0, yaw_rate=0.0))
        self.last_vx = 0.0
        self.last_vyaw = 0.0
        self.last_pub = time.monotonic()

    def shutdown(self):
        self.running = False
        self.publish_stop()  # 종료 시 안전 정지
        try:
            self.sock.close()
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = VrUdpTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

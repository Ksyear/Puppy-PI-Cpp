#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_status_sender (파이썬판)
==============================
로봇 상태(배터리 전압, Wi-Fi 신호세기, ROS 링크 상태)를 UDP 로 VR 헤드셋에
1초마다 전송. C++판(src/puppy_vr_control/src/robot_status_sender.cpp)과 동일 프로토콜.

프로토콜:
  1) Quest 가 UDP 5007 포트로 아무 패킷("hello")을 주기적으로 전송
  2) 로봇이 그 주소로 상태 문자열을 1Hz 푸시:
       "BAT:7400;BAT_AGE:0.4;RSSI:-52;UP:123"
     BAT=배터리(mV, 미수신 -1), BAT_AGE=배터리 토픽 경과초(3s 넘으면 드라이버 이상),
     RSSI=Wi-Fi dBm, UP=가동시간(초)
  3) Quest 쪽 판정: 이 패킷이 3초 이상 안 오면 "로봇 연결 끊김" 표시
"""

import socket
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16


class RobotStatusSender(Node):

    def __init__(self):
        super().__init__('robot_status_sender')

        self.bind_port = self.declare_parameter('bind_port', 5007).value
        self.client_ip = self.declare_parameter('client_ip', '').value
        self.client_port = self.declare_parameter('client_port', 5007).value
        self.send_period = self.declare_parameter('send_period', 1.0).value
        self.client_timeout_sec = self.declare_parameter('client_timeout_sec', 5.0).value
        self.wireless_if = self.declare_parameter('wireless_if', 'wlan0').value
        battery_topic = self.declare_parameter(
            'battery_topic', '/ros_robot_controller/battery').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.bind_port))
        self.sock.settimeout(0.2)

        self.lock = threading.Lock()
        self.client_addr = None
        self.last_hello = 0.0
        self.battery_mv = -1
        self.last_battery = 0.0
        self.start_time = time.monotonic()
        self.running = True

        if self.client_ip:
            self.client_addr = (self.client_ip, self.client_port)
        else:
            threading.Thread(target=self.hello_loop, daemon=True).start()

        self.battery_sub = self.create_subscription(
            UInt16, battery_topic, self.battery_cb, 1)
        self.timer = self.create_timer(self.send_period, self.send_status)

        self.get_logger().info(
            f'상태 전송 대기: UDP {self.bind_port} (배터리={battery_topic})')

    def battery_cb(self, msg):
        self.battery_mv = msg.data
        self.last_battery = time.monotonic()

    def hello_loop(self):
        while self.running:
            try:
                _, addr = self.sock.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break
            with self.lock:
                if addr != self.client_addr:
                    self.get_logger().info(f'상태 수신 클라이언트: {addr[0]}:{addr[1]}')
                self.client_addr = addr
                self.last_hello = time.monotonic()

    def read_rssi(self):
        """/proc/net/wireless 에서 신호세기(dBm) 파싱 (리눅스 전용)."""
        try:
            with open('/proc/net/wireless') as f:
                for line in f:
                    if self.wireless_if + ':' in line:
                        return int(float(line.split()[3]))
        except (OSError, ValueError, IndexError):
            pass
        return 0

    def send_status(self):
        now = time.monotonic()
        with self.lock:
            dest = self.client_addr
            if not self.client_ip and dest is not None:
                if now - self.last_hello > self.client_timeout_sec:
                    return  # Quest 쪽 hello 끊김 → 전송 중단
        if dest is None:
            return

        bat_age = -1.0 if self.battery_mv < 0 else now - self.last_battery
        uptime = int(now - self.start_time)
        msg = f'BAT:{self.battery_mv};BAT_AGE:{bat_age:.1f};RSSI:{self.read_rssi()};UP:{uptime}'
        try:
            self.sock.sendto(msg.encode(), dest)
        except OSError:
            return

        if bat_age > 3.0:
            self.get_logger().warn(
                f'배터리 토픽 {bat_age:.1f}s 무소식 — ros_robot_controller 상태 확인 필요',
                throttle_duration_sec=5.0)

    def shutdown(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusSender()
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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_status_sender (ROS1 Noetic 비상용)
========================================
ROS2판과 동일한 상태 전송 프로토콜 (UDP 5007, hello 자동 발견, 1Hz):
  "RSSI:-52;UP:123"
  - RSSI : Wi-Fi 신호세기(dBm, /proc/net/wireless). 읽기 실패 시 0
  - UP   : 노드 가동 시간(초)

배터리 표시는 제외됨: 이 로봇(RasAdapter 보드)은 배터리 전압을 소프트웨어로
읽을 수 없다 — I2C 스캔에 IMU(0x68)만 존재하고, 전압 ADC 는 뒷면 FND(숫자 표시)
전용 MCU 에만 연결돼 라즈베리파이에서 접근할 수 없다. 배터리 확인은 뒷면 FND
육안, 저전압 경고는 보드 자체 부저(6.8V 미만)가 담당한다.
자세한 근거는 noetic_fallback/README.md 참고.
"""

import socket
import threading
import time

import rospy


class RobotStatusSender(object):

    def __init__(self):
        rospy.init_node('robot_status_sender')

        self.bind_port = rospy.get_param('~bind_port', 5007)
        self.client_ip = rospy.get_param('~client_ip', '')
        self.client_port = rospy.get_param('~client_port', 5007)
        self.send_period = rospy.get_param('~send_period', 1.0)
        self.client_timeout_sec = rospy.get_param('~client_timeout_sec', 5.0)
        self.wireless_if = rospy.get_param('~wireless_if', 'wlan0')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.bind_port))
        self.sock.settimeout(0.2)

        self.lock = threading.Lock()
        self.client_addr = None
        self.last_hello = 0.0
        self.start_time = time.monotonic()
        self.running = True

        if self.client_ip:
            self.client_addr = (self.client_ip, self.client_port)
        else:
            threading.Thread(target=self.hello_loop, daemon=True).start()

        rospy.on_shutdown(self.shutdown)
        rospy.Timer(rospy.Duration(self.send_period), self.send_status)
        rospy.loginfo('상태 전송 대기(ROS1): UDP %d (Wi-Fi=%s)',
                      self.bind_port, self.wireless_if)

    def hello_loop(self):
        while self.running and not rospy.is_shutdown():
            try:
                _, addr = self.sock.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break
            with self.lock:
                if addr != self.client_addr:
                    rospy.loginfo('상태 수신 클라이언트: %s:%d', addr[0], addr[1])
                self.client_addr = addr
                self.last_hello = time.monotonic()

    def read_rssi(self):
        try:
            with open('/proc/net/wireless') as f:
                for line in f:
                    if self.wireless_if + ':' in line:
                        return int(float(line.split()[3]))
        except (OSError, ValueError, IndexError):
            pass
        return 0

    def send_status(self, _event):
        now = time.monotonic()
        with self.lock:
            dest = self.client_addr
            if not self.client_ip and dest is not None:
                if now - self.last_hello > self.client_timeout_sec:
                    return
        if dest is None:
            return
        uptime = int(now - self.start_time)

        msg = 'RSSI:%d;UP:%d' % (self.read_rssi(), uptime)
        try:
            self.sock.sendto(msg.encode(), dest)
        except OSError:
            pass

    def shutdown(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass


if __name__ == '__main__':
    RobotStatusSender()
    rospy.spin()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_status_sender (ROS1 Noetic 비상용)
========================================
ROS2판과 동일한 상태 전송 프로토콜 (UDP 5007, hello 자동 발견, 1Hz):
  "BAT:7400;BAT_AGE:0.4;RSSI:-52;UP:123"

주의: ROS1 이미지의 배터리 토픽 이름/타입은 이미지 버전에 따라 다를 수 있다.
  rostopic list | grep -i bat        # 이름 확인
  rostopic info <토픽>               # 타입 확인 (UInt16=mV / Float32=V 가정)
후 ~battery_topic / ~battery_type 파라미터로 맞출 것.
"""

import re
import socket
import threading
import time

import rospy

from std_msgs.msg import Float32, UInt16


class RobotStatusSender(object):

    def __init__(self):
        rospy.init_node('robot_status_sender')

        self.bind_port = rospy.get_param('~bind_port', 5007)
        self.client_ip = rospy.get_param('~client_ip', '')
        self.client_port = rospy.get_param('~client_port', 5007)
        self.send_period = rospy.get_param('~send_period', 1.0)
        self.client_timeout_sec = rospy.get_param('~client_timeout_sec', 5.0)
        self.wireless_if = rospy.get_param('~wireless_if', 'wlan0')
        # ''(기본값) = 자동 탐지: 이름에 bat/volt/power 가 들어간 std_msgs 토픽을 찾는다
        battery_topic = rospy.get_param('~battery_topic', '')
        battery_type = rospy.get_param('~battery_type', 'uint16')  # 'uint16'(mV) | 'float32'(V)

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

        if battery_topic:
            if battery_type == 'float32':
                rospy.Subscriber(battery_topic, Float32,
                                 lambda m: self.set_battery(int(m.data * 1000)))  # V -> mV
            else:
                rospy.Subscriber(battery_topic, UInt16, lambda m: self.set_battery(int(m.data)))
        else:
            threading.Thread(target=self.autodetect_battery, daemon=True).start()

        rospy.on_shutdown(self.shutdown)
        rospy.Timer(rospy.Duration(self.send_period), self.send_status)
        rospy.loginfo('상태 전송 대기(ROS1): UDP %d (배터리=%s)', self.bind_port, battery_topic)

    def autodetect_battery(self):
        """이름에 bat/volt/power 가 들어간 std_msgs 숫자 토픽을 찾아 구독."""
        import std_msgs.msg as std_msg_mod
        while self.running and not rospy.is_shutdown():
            try:
                topics = rospy.get_published_topics()
            except Exception:
                time.sleep(3)
                continue
            cands = [(t, ty) for t, ty in topics
                     if re.search(r'bat|volt|power', t, re.I) and ty.startswith('std_msgs/')]
            if cands:
                cands.sort(key=lambda x: ('bat' not in x[0].lower(), len(x[0])))
                topic, ty = cands[0]
                cls = getattr(std_msg_mod, ty.split('/')[1], None)
                if cls is not None:
                    rospy.Subscriber(topic, cls, self.any_battery_cb)
                    rospy.loginfo('배터리 토픽 자동 감지: %s (%s)', topic, ty)
                    return
            rospy.logwarn_throttle(
                15, '배터리 토픽 자동 탐지 실패 — 로봇에서 확인: rostopic list | grep -iE "bat|volt"\n'
                '  찾으면 실행 시 지정: run_vr.sh 대신 _battery_topic:=<토픽> 파라미터 사용')
            time.sleep(3)

    def any_battery_cb(self, m):
        """타입 불문 숫자 → mV 로 정규화 (100 미만이면 V 단위로 간주)."""
        try:
            v = float(getattr(m, 'data', -1))
        except (TypeError, ValueError):
            return
        self.set_battery(int(v * 1000) if 0 < v < 100 else int(v))

    def set_battery(self, mv):
        self.battery_mv = mv
        self.last_battery = time.monotonic()

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
        bat_age = -1.0 if self.battery_mv < 0 else now - self.last_battery
        uptime = int(now - self.start_time)
        msg = 'BAT:%d;BAT_AGE:%.1f;RSSI:%d;UP:%d' % (
            self.battery_mv, bat_age, self.read_rssi(), uptime)
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

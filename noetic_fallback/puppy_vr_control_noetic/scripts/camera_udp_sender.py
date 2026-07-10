#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_udp_sender (ROS1 Noetic 비상용)
======================================
ROS2판과 동일한 JPEG 청크 UDP 프로토콜 (5006, hello 자동 발견).
차이점: ROS1 usb_cam 의 압축 토픽은 /usb_cam/image_raw/compressed
(ROS2판의 /image_raw/compressed 와 이름이 다름 — 기본값으로 반영됨).

패킷: [uint32 frame_id][uint16 chunk_idx][uint16 chunk_cnt][uint32 frame_size][JPEG조각]
(빅엔디언, ROS2판과 동일 — Unity 수신 코드 재사용 가능)
"""

import socket
import struct
import threading
import time

import rospy

from sensor_msgs.msg import CompressedImage


class CameraUdpSender(object):

    def __init__(self):
        rospy.init_node('camera_udp_sender')

        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw/compressed')
        self.client_ip = rospy.get_param('~client_ip', '')
        self.client_port = rospy.get_param('~client_port', 5006)
        self.bind_port = rospy.get_param('~bind_port', 5006)
        self.max_fps = rospy.get_param('~max_fps', 15.0)
        self.chunk_size = rospy.get_param('~chunk_size', 1400)
        self.client_timeout_sec = rospy.get_param('~client_timeout_sec', 5.0)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.bind_port))
        self.sock.settimeout(0.2)

        self.lock = threading.Lock()
        self.client_addr = None
        self.last_hello = 0.0
        self.frame_counter = 0
        self.last_send = 0.0
        self.running = True

        if self.client_ip:
            self.client_addr = (self.client_ip, self.client_port)
        else:
            threading.Thread(target=self.hello_loop, daemon=True).start()

        rospy.Subscriber(self.image_topic, CompressedImage, self.image_cb, queue_size=1)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('영상 전송 대기(ROS1): %s -> UDP %d', self.image_topic, self.bind_port)

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
                    rospy.loginfo('클라이언트 발견: %s:%d', addr[0], addr[1])
                self.client_addr = addr
                self.last_hello = time.monotonic()

    def image_cb(self, msg):
        now = time.monotonic()
        if self.max_fps > 0 and now - self.last_send < 1.0 / self.max_fps:
            return
        with self.lock:
            dest = self.client_addr
            if not self.client_ip and dest is not None:
                if now - self.last_hello > self.client_timeout_sec:
                    return
        if dest is None:
            return

        jpeg = bytes(msg.data)
        total = len(jpeg)
        if total == 0:
            return
        count = (total + self.chunk_size - 1) // self.chunk_size
        if count > 65535:
            return
        frame_id = self.frame_counter & 0xFFFFFFFF
        self.frame_counter += 1
        for i in range(count):
            payload = jpeg[i * self.chunk_size:(i + 1) * self.chunk_size]
            header = struct.pack('!IHHI', frame_id, i, count, total)
            try:
                self.sock.sendto(header + payload, dest)
            except OSError:
                return
        self.last_send = now

    def shutdown(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass


if __name__ == '__main__':
    CameraUdpSender()
    rospy.spin()

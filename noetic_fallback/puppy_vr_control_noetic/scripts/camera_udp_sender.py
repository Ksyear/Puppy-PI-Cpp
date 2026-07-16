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

from sensor_msgs.msg import CompressedImage, Image


class CameraUdpSender(object):

    def __init__(self):
        rospy.init_node('camera_udp_sender')

        # ''(기본값) = 자동 탐지: 발행 중인 CompressedImage 토픽을 스스로 찾는다.
        # 이미지 버전마다 카메라 토픽 이름이 달라서 자동 탐지를 기본으로 함.
        self.image_topic = rospy.get_param('~image_topic', '')
        self.client_ip = rospy.get_param('~client_ip', '')
        self.client_port = rospy.get_param('~client_port', 5006)
        self.bind_port = rospy.get_param('~bind_port', 5006)
        self.max_fps = rospy.get_param('~max_fps', 15.0)
        self.chunk_size = rospy.get_param('~chunk_size', 1400)
        self.client_timeout_sec = rospy.get_param('~client_timeout_sec', 5.0)
        self.jpeg_quality = rospy.get_param('~jpeg_quality', 80)  # 원본 직접 인코딩 시 화질
        self.bridge = None

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

        if self.image_topic:
            rospy.Subscriber(self.image_topic, CompressedImage, self.image_cb, queue_size=1)
            rospy.loginfo('영상 전송 대기(ROS1): %s -> UDP %d', self.image_topic, self.bind_port)
        else:
            threading.Thread(target=self.autodetect_loop, daemon=True).start()
        rospy.on_shutdown(self.shutdown)

    def autodetect_loop(self):
        """발행 중인 CompressedImage 토픽을 찾아 구독. 없으면 원인/해결책을 로그로 안내."""
        preferred = '/usb_cam/image_raw/compressed'
        while self.running and not rospy.is_shutdown():
            try:
                topics = rospy.get_published_topics()
            except Exception:
                time.sleep(2)
                continue
            comp = [t for t, ty in topics if ty == 'sensor_msgs/CompressedImage']
            raw = [t for t, ty in topics if ty == 'sensor_msgs/Image']
            pick = None
            if preferred in comp:
                pick = preferred
            elif comp:
                comp.sort(key=lambda t: ('image' not in t.lower(), len(t)))
                pick = comp[0]
            if pick:
                self.image_topic = pick
                rospy.Subscriber(pick, CompressedImage, self.image_cb, queue_size=1)
                rospy.loginfo('카메라 토픽 자동 감지: %s -> UDP %d', pick, self.bind_port)
                return
            if raw:
                # 압축본이 없으면 원본을 구독해 노드가 직접 JPEG 인코딩 (추가 터미널 불필요)
                raw.sort(key=lambda t: (not t.startswith('/usb_cam'), 'image' not in t.lower(), len(t)))
                pick = raw[0]
                self.image_topic = pick
                rospy.Subscriber(pick, Image, self.raw_cb, queue_size=1, buff_size=2 ** 22)
                rospy.loginfo('압축 토픽 없음 → 원본 직접 JPEG 인코딩 전송: %s (quality=%d)',
                              pick, self.jpeg_quality)
                return
            else:
                rospy.logwarn_throttle(
                    10, '카메라 토픽이 전혀 없음 — 카메라 노드부터 실행:\n'
                    '  roslaunch puppy_bringup usb_cam.launch')
            time.sleep(2)

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

    def _dest_if_ready(self):
        """fps 제한/클라이언트 유효성 통과 시 목적지 반환, 아니면 None."""
        now = time.monotonic()
        if self.max_fps > 0 and now - self.last_send < 1.0 / self.max_fps:
            return None
        with self.lock:
            dest = self.client_addr
            if not self.client_ip and dest is not None:
                if now - self.last_hello > self.client_timeout_sec:
                    return None
        return dest

    def raw_cb(self, msg):
        """원본(sensor_msgs/Image) → JPEG 직접 인코딩 후 전송."""
        dest = self._dest_if_ready()
        if dest is None:
            return
        try:
            import cv2                      # 지연 임포트 (원본 경로일 때만 필요)
            from cv_bridge import CvBridge
            if self.bridge is None:
                self.bridge = CvBridge()
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)])
            if ok:
                self._send_jpeg(buf.tobytes(), dest)
        except Exception as e:
            rospy.logwarn_throttle(10, 'JPEG 인코딩 실패: %s', e)

    def image_cb(self, msg):
        dest = self._dest_if_ready()
        if dest is None:
            return
        self._send_jpeg(bytes(msg.data), dest)

    def _send_jpeg(self, jpeg, dest):
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
        self.last_send = time.monotonic()

    def shutdown(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass


if __name__ == '__main__':
    CameraUdpSender()
    rospy.spin()

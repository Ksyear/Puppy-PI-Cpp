#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VR 테스트 대시보드 (PC용, ROS 불필요)
=====================================
Quest 헤드셋 없이 VR 파이프라인 전체를 한 창에서 시험한다.
Quest 앱과 **완전히 동일한 UDP 프로토콜**로 동작하므로,
이 창에서 되는 것은 VR에서도 된다 (로봇은 둘을 구분하지 못함).

  [이 창]                                [로봇]
  가상 조이스틱/키보드 ──"X:..,Z:.."──▶ :5005 vr_udp_teleop → 로봇 이동
  ESTOP/RESUME 버튼 ────────────────▶ :5005
  영상 표시 ◀────── JPEG 청크 ─────── :5006 camera_udp_sender
  RSSI/연결 상태 ◀──── 상태 문자열 ── :5007 robot_status_sender

사용법:
  pip3 install pygame                    # 최초 1회 (이것만 필요)
  python3 tools/vr_test_dashboard.py --robot <로봇IP>

조작:
  마우스 드래그(조이스틱 원) 또는 W/A/S/D·방향키 = 이동
  SPACE = 긴급정지(ESTOP)   R = 해제(RESUME)
  P = 송신 일시정지(통신 끊김 시뮬레이션 — 로봇이 1초 내 자동 정지해야 정상)
  U = 카메라 AI 업스케일 x2 토글 (FSRCNN — 선택 기능, 아래 참고)
  ESC = 종료 (종료 시 정지 패킷 전송)

AI 업스케일 (선택, 노트북 쪽에서만 동작 — 로봇 부하/대역폭 영향 없음):
  pip3 install opencv-contrib-python     # dnn_superres 포함판이어야 함
  모델(FSRCNN_x2.pb, 40KB)은 첫 토글 때 자동 다운로드(tools/models/).
  로봇 핫스팟 접속 중엔 인터넷이 없으므로 미리 한 번 받아둘 것.

로봇 쪽 사전 조건: ROS2 는 vr_control.launch.py, ROS1(Noetic)은
noetic_fallback 의 vr_control.launch 가 떠 있으면 된다. (+ 카메라 노드)

셀프테스트(네트워크/GUI 없이 내부 로직 검증):
  python3 tools/vr_test_dashboard.py --selftest
"""

import argparse
import io
import math
import os
import socket
import struct
import sys
import threading
import time

CTRL_PORT = 5005
VIDEO_PORT = 5006
STATUS_PORT = 5007
MAP_PORT = 5008

# 로봇 쪽 vr_udp_teleop 기본 파라미터와 동일 (예상 명령 미리보기 계산용)
DEADZONE_DEG = 5.0
MAX_ANGLE_DEG = 45.0
MAX_SPEED_X = 15.0        # cm/s
MAX_YAW_RATE_DEG = 20.0   # deg/s
SPEED_STEP = 1.0
YAW_STEP_DEG = 2.0
INVERT_FORWARD = False
INVERT_TURN = True


# ─────────────────────── 프로토콜 로직 (셀프테스트 대상) ───────────────────────

class FrameAssembler:
    """camera_udp_sender 의 JPEG 청크를 프레임으로 재조립.
    헤더: !IHHI = frame_id, chunk_idx, chunk_cnt, frame_size (빅엔디언)"""

    def __init__(self):
        self.frame_id = None
        self.chunks = None
        self.received = 0

    def feed(self, pkt):
        if len(pkt) < 12:
            return None
        fid, idx, cnt, size = struct.unpack('!IHHI', pkt[:12])
        if cnt == 0 or idx >= cnt:
            return None
        if fid != self.frame_id or self.chunks is None or len(self.chunks) != cnt:
            # 새 프레임 시작 → 이전 미완성 프레임은 버림 (최신 우선)
            self.frame_id = fid
            self.chunks = [None] * cnt
            self.received = 0
        if self.chunks[idx] is None:
            self.chunks[idx] = pkt[12:]
            self.received += 1
            if self.received == cnt:
                data = b''.join(self.chunks)
                self.chunks = None
                if len(data) == size:
                    return data
        return None


def parse_status(text):
    """'RSSI:-52;UP:123' → dict"""
    out = {}
    for part in text.split(';'):
        key, _, val = part.partition(':')
        if key.strip():
            out[key.strip()] = val.strip()
    return out


def expected_robot_command(x_angle, z_angle):
    """로봇 vr_udp_teleop 과 동일한 변환으로 '예상 명령' 계산 (미리보기용)."""
    def norm(angle):
        if abs(angle) < DEADZONE_DEG:
            return 0.0
        return max(-1.0, min(1.0, angle / MAX_ANGLE_DEG))

    forward = norm(z_angle) * (-1.0 if INVERT_FORWARD else 1.0)
    turn = norm(x_angle) * (-1.0 if INVERT_TURN else 1.0)
    yaw_step = math.radians(YAW_STEP_DEG)
    max_yaw = math.radians(MAX_YAW_RATE_DEG)
    vx = round(forward * MAX_SPEED_X / SPEED_STEP) * SPEED_STEP
    vyaw = round(turn * max_yaw / yaw_step) * yaw_step
    vx = max(-MAX_SPEED_X, min(MAX_SPEED_X, vx))
    vyaw = max(-max_yaw, min(max_yaw, vyaw))
    return vx, vyaw


def selftest():
    # 청크 재조립: 3청크 프레임 정상 복원
    fa = FrameAssembler()
    jpeg = bytes(range(256)) * 20  # 5120B
    cnt = (len(jpeg) + 1399) // 1400
    out = None
    for i in range(cnt):
        payload = jpeg[i * 1400:(i + 1) * 1400]
        pkt = struct.pack('!IHHI', 7, i, cnt, len(jpeg)) + payload
        out = fa.feed(pkt) or out
    assert out == jpeg, '재조립 실패'
    # 청크 유실 → 다음 프레임으로 넘어가면 이전 것은 버려짐
    fa2 = FrameAssembler()
    fa2.feed(struct.pack('!IHHI', 1, 0, 2, 10) + b'x' * 5)          # 프레임1 절반만
    r = fa2.feed(struct.pack('!IHHI', 2, 0, 1, 3) + b'abc')          # 프레임2 완성
    assert r == b'abc', '유실 처리 실패'
    # 상태 파싱
    st = parse_status('RSSI:-52;UP:123')
    assert st['RSSI'] == '-52' and st['UP'] == '123'
    # 예상 명령: Z=+45° 풀틸트 → 전진 15cm/s / 데드존 안 → 0
    assert expected_robot_command(0.0, 45.0) == (15.0, 0.0)
    assert expected_robot_command(3.0, 3.0) == (0.0, 0.0)
    vx, vyaw = expected_robot_command(-45.0, 0.0)   # X=-45 → +yaw(좌회전, invert_turn 기본값 기준)
    assert vx == 0.0 and abs(vyaw - math.radians(20)) < 1e-9
    print('selftest OK (청크 재조립 / 상태 파싱 / 명령 미리보기)')


class SuperRes(object):
    """카메라 프레임 x2 AI 업스케일 (FSRCNN, OpenCV dnn_superres).

    왜 노트북에서 하나: 로봇(Pi4)은 SLAM+카메라로 CPU 여유가 없고, 로봇에서
    키우면 JPEG 가 커져 Wi-Fi 대역폭만 늘어난다. "작게 보내고 화면에서 키운다".

    선택 기능 — opencv-contrib-python + 모델 파일이 있을 때만 켜진다.
    준비가 안 되면 reason 에 안내문을 담고 꺼진 채로 남는다 (원본 그대로 표시).
    """
    MODEL_FILE = 'FSRCNN_x2.pb'   # ~40KB, 공식 배포처 (OpenCV dnn_superres 표준 모델)
    MODEL_URL = ('https://github.com/Saafke/FSRCNN_Tensorflow/'
                 'raw/master/models/FSRCNN_x2.pb')

    def __init__(self):
        self.sr = None        # cv2 DnnSuperResImpl (준비 완료 시)
        self.cv2 = None
        self.np = None
        self.reason = ''      # 비활성 사유 (UI 에 표시)
        self.ms = 0.0         # 마지막 프레임 처리 시간 (UI 에 표시)

    def model_path(self):
        return os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'models', self.MODEL_FILE)

    def try_init(self):
        """cv2 확인 + 모델 준비(없으면 다운로드 시도). 성공 시 True."""
        if self.sr is not None:
            return True
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.reason = 'pip3 install opencv-contrib-python'
            return False
        if not hasattr(cv2, 'dnn_superres'):
            self.reason = 'contrib 판 필요: pip3 install opencv-contrib-python'
            return False
        path = self.model_path()
        if not os.path.exists(path):
            # 첫 사용 시 자동 다운로드. 로봇 핫스팟 접속 중엔 인터넷이 없어
            # 실패한다 → 안내문을 남기고 다음에(인터넷 될 때) 다시 시도.
            try:
                import urllib.request
                os.makedirs(os.path.dirname(path), exist_ok=True)
                urllib.request.urlretrieve(self.MODEL_URL, path + '.part')
                os.replace(path + '.part', path)
            except Exception:
                self.reason = '모델 다운로드 실패 — 인터넷 연결 후 U 재시도'
                return False
        try:
            sr = cv2.dnn_superres.DnnSuperResImpl_create()
            sr.readModel(path)
            sr.setModel('fsrcnn', 2)
        except Exception as e:
            self.reason = '모델 로드 실패: %s' % e
            return False
        self.cv2, self.np, self.sr = cv2, np, sr
        self.reason = ''
        return True

    def process(self, jpeg):
        """JPEG bytes → 업스케일된 (rgb_bytes, w, h). 실패 시 None (원본 표시)."""
        if self.sr is None:
            return None
        try:
            t0 = time.perf_counter()
            arr = self.np.frombuffer(jpeg, self.np.uint8)
            bgr = self.cv2.imdecode(arr, self.cv2.IMREAD_COLOR)
            if bgr is None:
                return None
            up = self.sr.upsample(bgr)
            rgb = self.cv2.cvtColor(up, self.cv2.COLOR_BGR2RGB)
            self.ms = (time.perf_counter() - t0) * 1000.0
            return rgb.tobytes(), up.shape[1], up.shape[0]
        except Exception:
            return None


# ─────────────────────────────── 대시보드 본체 ───────────────────────────────

class Dashboard:
    SEND_HZ = 20.0  # Unity UDP_Joystick_Sender 와 동일

    def __init__(self, robot_ip, max_tilt=30.0, ramp=0.35):
        self.robot = robot_ip
        self.max_tilt = max_tilt   # 풀 조작 시 보낼 기울기(°) — 45=로봇 최대속도
        self.ramp = ramp           # 0→풀 기울기 도달 시간(초) — 급출발/급정지 방지
        self.ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video_sock.settimeout(0.2)
        self.status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_sock.settimeout(0.2)
        self.map_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.map_sock.settimeout(0.2)

        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.latest_sr = None        # 업스케일된 최신 프레임 (rgb_bytes, w, h)
        self.latest_map = None
        self.video_frames = 0
        self.video_fps = 0.0
        self.status = {}
        self.last_status_time = 0.0
        self.estop = False
        self.tx_paused = False
        self.running = True

        # AI 업스케일 (U 키 토글) — 수신 스레드에서 처리해 조종 루프를 막지 않음
        self.sr = SuperRes()
        self.sr_enabled = False
        self.sr_loading = False

        threading.Thread(target=self.video_loop, daemon=True).start()
        threading.Thread(target=self.status_loop, daemon=True).start()
        threading.Thread(target=self.map_loop, daemon=True).start()
        threading.Thread(target=self.fps_loop, daemon=True).start()

    # hello 를 1초마다 보내 로봇의 자동 발견 대상이 되고, 응답 스트림을 수신
    def video_loop(self):
        last_hello = 0.0
        fa = FrameAssembler()
        while self.running:
            now = time.monotonic()
            if now - last_hello > 1.0:
                try:
                    self.video_sock.sendto(b'hello', (self.robot, VIDEO_PORT))
                except OSError:
                    pass
                last_hello = now
            try:
                pkt, _ = self.video_sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            jpeg = fa.feed(pkt)
            if jpeg:
                # SR 은 여기(수신 스레드)에서: UI/조종 루프가 안 밀린다.
                # SR 이 프레임 간격보다 느리면 커널 소켓버퍼가 넘치며 오래된
                # 청크가 자연 폐기되어 fps 만 떨어진다 (지연 누적 없음).
                sr_frame = self.sr.process(jpeg) if self.sr_enabled else None
                with self.lock:
                    self.latest_jpeg = jpeg
                    self.latest_sr = sr_frame
                    self.video_frames += 1

    def enable_sr(self):
        """U 키: 백그라운드에서 초기화 (모델 다운로드가 UI 를 얼리지 않도록)."""
        if self.sr.try_init():
            self.sr_enabled = True
        self.sr_loading = False

    def map_loop(self):
        """지도 이미지 수신 (영상과 같은 청크 프로토콜, 포트 5008, hello 자동 발견)."""
        last_hello = 0.0
        fa = FrameAssembler()
        while self.running:
            now = time.monotonic()
            if now - last_hello > 1.0:
                try:
                    self.map_sock.sendto(b'hello', (self.robot, MAP_PORT))
                except OSError:
                    pass
                last_hello = now
            try:
                pkt, _ = self.map_sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            jpeg = fa.feed(pkt)
            if jpeg:
                with self.lock:
                    self.latest_map = jpeg

    def status_loop(self):
        last_hello = 0.0
        while self.running:
            now = time.monotonic()
            if now - last_hello > 1.0:
                try:
                    self.status_sock.sendto(b'hello', (self.robot, STATUS_PORT))
                except OSError:
                    pass
                last_hello = now
            try:
                pkt, _ = self.status_sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                break
            with self.lock:
                self.status = parse_status(pkt.decode('utf-8', errors='replace'))
                self.last_status_time = time.monotonic()

    def fps_loop(self):
        while self.running:
            time.sleep(1.0)
            with self.lock:
                self.video_fps = self.video_frames
                self.video_frames = 0

    def send_ctrl(self, text):
        try:
            self.ctrl_sock.sendto(text.encode(), (self.robot, CTRL_PORT))
        except OSError:
            pass

    def run(self):
        import pygame  # GUI 가 필요할 때만 import (selftest 는 pygame 불필요)
        pygame.init()
        MIN_W, MIN_H = 640, 480
        # 작은 노트북에서도 들어가도록 기본 창을 줄이고 리사이즈 허용
        screen = pygame.display.set_mode((960, 620), pygame.RESIZABLE)
        pygame.display.set_caption('PuppyPi VR Test Dashboard  (robot: %s)' % self.robot)
        clock = pygame.time.Clock()

        # 폰트는 창 크기에 맞춰 만들고 캐시 (매 프레임 재생성 방지)
        _font_cache = {}

        def get_font(size, bold=False):
            f = _font_cache.get((size, bold))
            if f is None:
                f = pygame.font.SysFont('monospace', size, bold=bold)
                _font_cache[(size, bold)] = f
            return f

        knob = [0.0, 0.0]      # 목표 입력 (오른쪽+, 위쪽+) — -1..1
        sx, sy = 0.0, 0.0      # 램프가 적용된 실제 송신값
        dragging = False
        send_acc = 0.0

        while self.running:
            dt = clock.tick(60) / 1000.0
            W, H = screen.get_size()

            # ── 창 크기에 맞춘 반응형 레이아웃 (매 프레임 계산) ──
            m = 10
            right_w = min(max(int(W * 0.34), 210), 460)   # 오른쪽 상태/조이스틱 열
            left_w = max(160, W - right_w - 3 * m)         # 왼쪽 영상/지도 열
            avail_h = H - 3 * m
            cam_h = avail_h // 2
            map_h = avail_h - cam_h
            cam_rect = pygame.Rect(m, m, left_w, cam_h)
            map_rect = pygame.Rect(m, 2 * m + cam_h, left_w, map_h)

            rx = left_w + 2 * m
            fs = max(11, min(16, right_w // 22))           # 본문 폰트
            fs_big = max(15, min(24, right_w // 13))        # 배너 폰트
            font = get_font(fs)
            big = get_font(fs_big, bold=True)
            lh = fs + 6

            banner_y = m
            joy_radius = max(45, min(right_w // 2 - 15, H // 6))
            joy_cx = rx + right_w // 2
            joy_cy = banner_y + fs_big + 16 + joy_radius
            joy_center = (joy_cx, joy_cy)
            text_x = rx
            text_y0 = joy_cy + joy_radius + fs + 18

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    self.running = False
                elif ev.type == pygame.VIDEORESIZE:
                    screen = pygame.display.set_mode(
                        (max(MIN_W, ev.w), max(MIN_H, ev.h)), pygame.RESIZABLE)
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        self.running = False
                    elif ev.key == pygame.K_SPACE:
                        self.estop = True
                        knob = [0.0, 0.0]
                        sx, sy = 0.0, 0.0
                        for _ in range(3):
                            self.send_ctrl('ESTOP')
                    elif ev.key == pygame.K_r:
                        self.estop = False
                        self.send_ctrl('RESUME')
                    elif ev.key == pygame.K_p:
                        self.tx_paused = not self.tx_paused
                    elif ev.key == pygame.K_u:
                        if self.sr_enabled:
                            self.sr_enabled = False
                        elif not self.sr_loading:
                            self.sr_loading = True
                            threading.Thread(target=self.enable_sr, daemon=True).start()
                elif ev.type == pygame.MOUSEBUTTONDOWN:
                    dx = ev.pos[0] - joy_center[0]
                    dy = ev.pos[1] - joy_center[1]
                    if dx * dx + dy * dy <= joy_radius * joy_radius:
                        dragging = True
                elif ev.type == pygame.MOUSEBUTTONUP:
                    dragging = False
                    knob = [0.0, 0.0]   # 놓으면 중립 (스프링 리턴)
                elif ev.type == pygame.MOUSEMOTION and dragging:
                    dx = (ev.pos[0] - joy_center[0]) / joy_radius
                    dy = -(ev.pos[1] - joy_center[1]) / joy_radius
                    mag = math.hypot(dx, dy)
                    if mag > 1.0:
                        dx, dy = dx / mag, dy / mag
                    knob = [dx, dy]

            # 키보드는 풀틸트 디지털 입력 (마우스 드래그보다 우선)
            keys = pygame.key.get_pressed()
            kx = (keys[pygame.K_d] or keys[pygame.K_RIGHT]) - (keys[pygame.K_a] or keys[pygame.K_LEFT])
            ky = (keys[pygame.K_w] or keys[pygame.K_UP]) - (keys[pygame.K_s] or keys[pygame.K_DOWN])
            tx, ty = (float(kx), float(ky)) if (kx or ky) else (knob[0], knob[1])

            # 부드러운 가감속(램프): 목표 기울기까지 ramp 초에 걸쳐 서서히 도달
            # — 키를 누르는 순간 최대속도가 확 나가서 로봇이 홱 움직이는 것 방지
            step = dt / max(self.ramp, 0.01)
            sx += max(-step, min(step, tx - sx))
            sy += max(-step, min(step, ty - sy))

            # 낮은 기울기에서 미세 조종이 쉽도록 완만한 곡선(expo)
            def expo(v):
                return v * (0.5 + 0.5 * abs(v))
            x_angle = self.max_tilt * expo(sx)
            z_angle = self.max_tilt * expo(sy)

            # 20Hz 송신 (Unity sendRate 와 동일). P 로 끊김 시뮬레이션 가능
            send_acc += dt
            if send_acc >= 1.0 / self.SEND_HZ:
                send_acc = 0.0
                if not self.tx_paused and not self.estop:
                    self.send_ctrl('X:%.1f,Z:%.1f' % (x_angle, z_angle))

            # ── 그리기 ──
            screen.fill((24, 26, 30))

            # 카메라 영상(위) + 라이다 지도(아래) — 둘 다 항상 화면에 표시
            with self.lock:
                jpeg_cam = self.latest_jpeg
                sr_cam = self.latest_sr if self.sr_enabled else None
                jpeg_map = self.latest_map
                fps = self.video_fps
                status = dict(self.status)
                status_age = time.monotonic() - self.last_status_time if self.last_status_time else 1e9

            def make_surface(jpeg, sr_frame=None):
                """SR 결과가 있으면 그걸, 아니면 JPEG 디코드. 실패 시 None."""
                try:
                    if sr_frame:
                        rgb, w, h = sr_frame
                        return pygame.image.frombuffer(rgb, (w, h), 'RGB')
                    if jpeg:
                        return pygame.image.load(io.BytesIO(jpeg))
                except Exception:
                    pass
                return None

            def draw_panel(rect, surf, label, label_col, smooth, wait_msg):
                if surf:
                    try:
                        iw, ih = surf.get_size()
                        # 종횡비 보존(레터박스): 카메라는 안 찌그러지고 정사각 지도도 형태 유지
                        s = min(rect.w / iw, rect.h / ih)
                        tw, th = max(1, int(iw * s)), max(1, int(ih * s))
                        scaler = pygame.transform.smoothscale if smooth else pygame.transform.scale
                        img = scaler(surf, (tw, th))   # 지도는 nearest 로 격자를 또렷하게
                        screen.blit(img, (rect.x + (rect.w - tw) // 2,
                                          rect.y + (rect.h - th) // 2))
                    except Exception:
                        pass
                else:
                    pygame.draw.rect(screen, (40, 42, 48), rect)
                    screen.blit(font.render(wait_msg, True, (140, 140, 140)),
                                (rect.x + 14, rect.y + rect.h // 2 - fs))
                pygame.draw.rect(screen, (70, 74, 84), rect, 2)
                screen.blit(font.render(label, True, label_col), (rect.x + 8, rect.y + 6))

            cam_label = 'CAMERA  :5006' + ('  [SR x2]' if sr_cam else '')
            draw_panel(cam_rect, make_surface(jpeg_cam, sr_cam), cam_label,
                       (150, 150, 150), True, 'NO VIDEO (:5006)')
            draw_panel(map_rect, make_surface(jpeg_map), 'LiDAR MAP  :5008',
                       (240, 210, 90), False, 'NO MAP (use_mapping:=true)')

            # 가상 조이스틱
            pygame.draw.circle(screen, (46, 50, 58), joy_center, joy_radius)
            pygame.draw.circle(screen, (90, 96, 110), joy_center, joy_radius, 2)
            knob_r = max(8, joy_radius // 6)
            kx_px = int(joy_cx + sx * (joy_radius - knob_r))
            ky_px = int(joy_cy - sy * (joy_radius - knob_r))
            pygame.draw.circle(screen, (200, 60, 60) if self.estop else (80, 160, 255), (kx_px, ky_px), knob_r)
            lbl = font.render('drag / WASD', True, (150, 150, 150))
            screen.blit(lbl, (joy_cx - lbl.get_width() // 2, joy_cy + joy_radius + 6))

            # 상태 패널
            robot_link = status_age < 3.0
            vx, vyaw = expected_robot_command(x_angle, z_angle)

            def line(i, text, color=(210, 210, 210)):
                screen.blit(font.render(text, True, color), (text_x, text_y0 + i * lh))

            link_col = (90, 220, 120) if robot_link else (230, 80, 80)
            line(0, 'LINK  : %s' % ('OK (%.1fs)' % status_age if robot_link else 'LOST'), link_col)
            line(1, 'RSSI  : %s  UP:%ss' % (status.get('RSSI', '--'), status.get('UP', '--')))
            line(2, 'VIDEO : %d fps' % fps, (90, 220, 120) if fps > 0 else (150, 150, 150))
            line(3, 'TX    : %s' % ('PAUSED (P)' if self.tx_paused else '%.0fHz ->:%d' % (self.SEND_HZ, CTRL_PORT)),
                 (240, 180, 60) if self.tx_paused else (210, 210, 210))
            line(4, 'ANGLE : X=%+5.1f Z=%+5.1f' % (x_angle, z_angle))
            line(5, 'CMD   : vx=%+5.1f yaw=%+5.2f' % (vx, vyaw), (120, 190, 255))
            if self.sr_loading:
                line(6, 'SR    : loading...', (240, 180, 60))
            elif self.sr_enabled:
                line(6, 'SR    : ON x2 (%.0fms)' % self.sr.ms, (90, 220, 120))
            elif self.sr.reason:
                line(6, 'SR    : %s' % self.sr.reason, (230, 80, 80))
            else:
                line(6, 'SR    : OFF (U)', (150, 150, 150))
            line(8, 'SPACE=ESTOP  R=resume', (150, 150, 150))
            line(9, 'P=pauseTX U=SR ESC=quit', (150, 150, 150))

            if self.estop:
                screen.blit(big.render('E-STOP (R)', True, (255, 70, 70)), (rx, banner_y))
            elif self.tx_paused:
                screen.blit(big.render('TX PAUSED (P)', True, (240, 180, 60)), (rx, banner_y))
            else:
                screen.blit(big.render('DRIVING', True, (90, 220, 120)), (rx, banner_y))

            pygame.display.flip()

        # 종료: 정지 패킷을 확실히 보내고 소켓 정리
        for _ in range(3):
            self.send_ctrl('X:0.0,Z:0.0')
            time.sleep(0.02)
        pygame.quit()
        for s in (self.ctrl_sock, self.video_sock, self.status_sock, self.map_sock):
            try:
                s.close()
            except OSError:
                pass


def main():
    ap = argparse.ArgumentParser(description='PuppyPi VR 테스트 대시보드 (Quest 없이 전체 파이프라인 시험)')
    ap.add_argument('--robot', default='192.168.0.100', help='로봇 IP')
    ap.add_argument('--max-tilt', type=float, default=45.0,
                    help='풀 조작 시 기울기(°). 실기기 확정 기본값 45 (Unity 조이스틱과 동일 범위)')
    ap.add_argument('--ramp', type=float, default=0.35,
                    help='0→풀 기울기 도달 시간(초). 클수록 부드럽고 작을수록 민첩 (기본 0.35)')
    ap.add_argument('--selftest', action='store_true', help='네트워크/GUI 없이 내부 로직 검증')
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return

    try:
        import pygame  # noqa: F401
    except ImportError:
        print('pygame 이 필요합니다:  pip3 install pygame')
        sys.exit(1)

    print('대시보드 시작 — 로봇: %s (조종:%d 영상:%d 상태:%d 지도:%d)' % (
        args.robot, CTRL_PORT, VIDEO_PORT, STATUS_PORT, MAP_PORT))
    print('  카메라(위) + 라이다 지도(아래)가 한 화면에 함께 표시됨 '
          '(지도는 로봇에서 use_mapping:=true 실행 필요)')
    Dashboard(args.robot, max_tilt=args.max_tilt, ramp=args.ramp).run()


if __name__ == '__main__':
    main()

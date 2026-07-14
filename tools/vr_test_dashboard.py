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
  배터리/RSSI/연결 ◀── 상태 문자열 ── :5007 robot_status_sender

사용법:
  pip3 install pygame                    # 최초 1회 (이것만 필요)
  python3 tools/vr_test_dashboard.py --robot <로봇IP>

조작:
  마우스 드래그(조이스틱 원) 또는 W/A/S/D·방향키 = 이동
  SPACE = 긴급정지(ESTOP)   R = 해제(RESUME)
  P = 송신 일시정지(통신 끊김 시뮬레이션 — 로봇이 1초 내 자동 정지해야 정상)
  ESC = 종료 (종료 시 정지 패킷 전송)

로봇 쪽 사전 조건: ROS2 는 vr_control.launch.py, ROS1(Noetic)은
noetic_fallback 의 vr_control.launch 가 떠 있으면 된다. (+ 카메라 노드)

셀프테스트(네트워크/GUI 없이 내부 로직 검증):
  python3 tools/vr_test_dashboard.py --selftest
"""

import argparse
import io
import math
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
    """'BAT:7400;BAT_AGE:0.4;RSSI:-52;UP:123' → dict"""
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
    st = parse_status('BAT:7400;BAT_AGE:0.4;RSSI:-52;UP:123')
    assert st['BAT'] == '7400' and st['RSSI'] == '-52'
    # 예상 명령: Z=+45° 풀틸트 → 전진 15cm/s / 데드존 안 → 0
    assert expected_robot_command(0.0, 45.0) == (15.0, 0.0)
    assert expected_robot_command(3.0, 3.0) == (0.0, 0.0)
    vx, vyaw = expected_robot_command(-45.0, 0.0)   # X=-45 → +yaw(좌회전, invert_turn 기본값 기준)
    assert vx == 0.0 and abs(vyaw - math.radians(20)) < 1e-9
    print('selftest OK (청크 재조립 / 상태 파싱 / 명령 미리보기)')


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
        self.latest_map = None
        self.view_map = False
        self.video_frames = 0
        self.video_fps = 0.0
        self.status = {}
        self.last_status_time = 0.0
        self.estop = False
        self.tx_paused = False
        self.running = True

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
                with self.lock:
                    self.latest_jpeg = jpeg
                    self.video_frames += 1

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
        screen = pygame.display.set_mode((1020, 540))
        pygame.display.set_caption('PuppyPi VR Test Dashboard  (robot: %s)' % self.robot)
        clock = pygame.time.Clock()
        font = pygame.font.SysFont('monospace', 16)
        big = pygame.font.SysFont('monospace', 26, bold=True)

        joy_center = (830, 190)
        joy_radius = 95
        knob = [0.0, 0.0]      # 목표 입력 (오른쪽+, 위쪽+) — -1..1
        sx, sy = 0.0, 0.0      # 램프가 적용된 실제 송신값
        dragging = False
        send_acc = 0.0

        while self.running:
            dt = clock.tick(60) / 1000.0
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    self.running = False
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
                    elif ev.key == pygame.K_m:
                        self.view_map = not self.view_map
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

            # 영상 (좌측 640x480)
            with self.lock:
                jpeg = self.latest_map if self.view_map else self.latest_jpeg
                fps = self.video_fps
                status = dict(self.status)
                status_age = time.monotonic() - self.last_status_time if self.last_status_time else 1e9
            video_rect = pygame.Rect(10, 10, 640, 480)
            if jpeg:
                try:
                    img = pygame.image.load(io.BytesIO(jpeg))
                    img = pygame.transform.smoothscale(img, (640, 480))
                    screen.blit(img, video_rect.topleft)
                except Exception:
                    pass
            else:
                pygame.draw.rect(screen, (40, 42, 48), video_rect)
                wait_msg = 'NO MAP (waiting :5008... use_mapping:=true?)' if self.view_map \
                    else 'NO VIDEO (waiting :5006...)'
                screen.blit(font.render(wait_msg, True, (140, 140, 140)), (170, 240))
            pygame.draw.rect(screen, (70, 74, 84), video_rect, 2)
            screen.blit(font.render(
                'MAP  [M: camera]' if self.view_map else 'CAMERA  [M: map]',
                True, (240, 210, 90) if self.view_map else (150, 150, 150)), (18, 16))

            # 가상 조이스틱
            pygame.draw.circle(screen, (46, 50, 58), joy_center, joy_radius)
            pygame.draw.circle(screen, (90, 96, 110), joy_center, joy_radius, 2)
            kx_px = int(joy_center[0] + sx * (joy_radius - 18))
            ky_px = int(joy_center[1] - sy * (joy_radius - 18))
            pygame.draw.circle(screen, (200, 60, 60) if self.estop else (80, 160, 255), (kx_px, ky_px), 16)
            screen.blit(font.render('drag / WASD', True, (150, 150, 150)), (joy_center[0] - 50, joy_center[1] + joy_radius + 8))

            # 상태 패널
            robot_link = status_age < 3.0
            bat_mv = int(status.get('BAT', -1)) if status.get('BAT', '').lstrip('-').isdigit() else -1
            vx, vyaw = expected_robot_command(x_angle, z_angle)

            def line(i, text, color=(210, 210, 210)):
                screen.blit(font.render(text, True, color), (668, 320 + i * 22))

            link_col = (90, 220, 120) if robot_link else (230, 80, 80)
            line(0, 'ROBOT LINK : %s' % ('OK (%.1fs)' % status_age if robot_link else 'LOST'), link_col)
            if bat_mv > 0:
                bat_col = (90, 220, 120) if bat_mv >= 7000 else (240, 180, 60) if bat_mv >= 6600 else (230, 80, 80)
                line(1, 'BATTERY    : %.2f V %s' % (bat_mv / 1000.0, '(LOW!)' if bat_mv < 7000 else ''), bat_col)
            else:
                line(1, 'BATTERY    : --', (150, 150, 150))
            line(2, 'WIFI RSSI  : %s dBm   UPTIME: %ss' % (status.get('RSSI', '--'), status.get('UP', '--')))
            line(3, 'VIDEO      : %d fps' % fps, (90, 220, 120) if fps > 0 else (150, 150, 150))
            line(4, 'TX         : %s' % ('PAUSED (P)' if self.tx_paused else '%.0f Hz -> :%d' % (self.SEND_HZ, CTRL_PORT)),
                 (240, 180, 60) if self.tx_paused else (210, 210, 210))
            line(5, 'SEND ANGLE : X=%+6.1f  Z=%+6.1f' % (x_angle, z_angle))
            line(6, 'EXPECT CMD : vx=%+5.1f cm/s  yaw=%+5.2f rad/s' % (vx, vyaw), (120, 190, 255))
            line(8, '[SPACE] ESTOP  [R] resume  [P] pause-TX  [M] map  [ESC] quit', (150, 150, 150))

            if self.estop:
                screen.blit(big.render('E-STOP ACTIVE  (press R)', True, (255, 70, 70)), (660, 20))
            elif self.tx_paused:
                screen.blit(big.render('TX PAUSED - watchdog test', True, (240, 180, 60)), (660, 20))
            else:
                screen.blit(big.render('DRIVING ENABLED', True, (90, 220, 120)), (660, 20))

            # 로봇 쪽 저전압 보호가 발동하면 최우선으로 경고
            if status.get('LOW') == '1':
                screen.blit(big.render('LOW BATTERY - CHARGE NOW!', True, (255, 70, 70)), (660, 52))

            pygame.display.flip()

        # 종료: 정지 패킷을 확실히 보내고 소켓 정리
        for _ in range(3):
            self.send_ctrl('X:0.0,Z:0.0')
            time.sleep(0.02)
        pygame.quit()
        for s in (self.ctrl_sock, self.video_sock, self.status_sock):
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

    print('대시보드 시작 — 로봇: %s (조종:%d 영상:%d 상태:%d)' % (
        args.robot, CTRL_PORT, VIDEO_PORT, STATUS_PORT))
    Dashboard(args.robot, max_tilt=args.max_tilt, ramp=args.ramp).run()


if __name__ == '__main__':
    main()

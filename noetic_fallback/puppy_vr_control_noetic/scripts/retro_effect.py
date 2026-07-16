#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
retro_effect — 옛날 TV(CRT) 스타일 영상 효과 (rospy 비의존 순수 로직)
====================================================================
camera_udp_sender 가 영상을 UDP 로 내보내기 직전에 적용한다.
AI 모델이 아니라 고전 영상처리(numpy/OpenCV)라 라즈베리파이에서 실시간으로 돈다.

구성 요소 (libretro CRT 셰이더의 표준 구성 중 Pi 에서 싼 것만 채택):
  1) 잔상(phosphor persistence): out = max(현재, 이전출력 × ghost)
     — 밝은 픽셀이 여러 프레임에 걸쳐 서서히 어두워지며 꼬리를 남긴다
  2) 업스케일 (bilinear ×2 — AI 아님)
  3) 색수차(chroma shift): R/B 채널을 좌우로 어긋나게 — 아날로그 색번짐
  4) 주사선(scanline) + 비네트(vignette): 미리 계산해 캐시한 마스크 1회 곱
     (두 효과를 한 마스크로 합쳐 Pi 에서 큰 곱셈이 1번만 일어나게)

사용:
    fx = RetroEffect(ghost=0.65, scanline=0.35, vignette=0.35, chroma=2, upscale=2.0)
    out_bgr = fx.apply(bgr)          # fx.ms 에 처리시간(ms) 기록

셀프테스트 (로봇에서도 그대로 실행해 실측 가능):
    python3 retro_effect.py
"""

import time

import cv2
import numpy as np


class RetroEffect(object):

    def __init__(self, ghost=0.65, scanline=0.35, vignette=0.35, chroma=2, upscale=2.0):
        self.ghost = float(max(0.0, min(0.95, ghost)))       # 잔상 유지율/프레임 (0=끔)
        self.scanline = float(max(0.0, min(0.9, scanline)))  # 주사선 어둡기 (0=끔)
        self.vignette = float(max(0.0, min(0.9, vignette)))  # 모서리 어둡기 (0=끔)
        self.chroma = int(max(0, chroma))                    # 색수차 픽셀 (0=끔)
        self.upscale = float(max(1.0, upscale))              # 1.0=원본 해상도 유지
        self.prev = None          # 잔상 버퍼 (float32, 입력 해상도)
        self._mask = None         # 주사선×비네트 결합 마스크 (출력 해상도, 캐시)
        self._mask_key = None
        self.ms = 0.0             # 마지막 apply 처리시간 (ms)

    def _combined_mask(self, h, w):
        """주사선 + 비네트를 하나의 곱셈 마스크로 (크기별 1회 계산 후 캐시)."""
        key = (h, w)
        if self._mask_key == key:
            return self._mask
        mask = np.ones((h, w), np.float32)
        if self.scanline > 0.0:
            period = max(2, int(round(self.upscale)))
            mask[period - 1::period] *= (1.0 - self.scanline)
        if self.vignette > 0.0:
            yy = np.linspace(-1.0, 1.0, h, dtype=np.float32)[:, None]
            xx = np.linspace(-1.0, 1.0, w, dtype=np.float32)[None, :]
            r2 = xx * xx + yy * yy
            # 중앙(r2<0.25)은 원래 밝기, 모서리로 갈수록 어두워짐
            mask *= 1.0 - self.vignette * np.clip(r2 - 0.25, 0.0, 1.0)
        self._mask = mask[:, :, None]      # (h, w, 1) — 채널 브로드캐스트
        self._mask_key = key
        return self._mask

    def apply(self, bgr):
        """BGR uint8 → 효과 적용된 BGR uint8. 입력 크기가 바뀌면 잔상만 리셋."""
        t0 = time.perf_counter()
        cur = bgr.astype(np.float32)

        # 1) 잔상 — 밝은 픽셀이 ghost 비율로 남으며 프레임마다 감쇠 (CRT phosphor)
        if self.prev is None or self.prev.shape != cur.shape:
            self.prev = cur
        else:
            cur = np.maximum(cur, self.prev * self.ghost)
            self.prev = cur

        # 2) 업스케일. upscale=1 이면 잔상 버퍼(prev)와 메모리를 공유하므로
        #    아래의 제자리 수정이 버퍼를 오염시키지 않도록 복사한다.
        if self.upscale > 1.0:
            cur = cv2.resize(cur, None, fx=self.upscale, fy=self.upscale,
                             interpolation=cv2.INTER_LINEAR)
        else:
            cur = cur.copy()

        # 3) 색수차 — R 은 오른쪽, B 는 왼쪽으로 어긋나게 (아날로그 색번짐)
        c = self.chroma
        if c > 0:
            cur[:, c:, 2] = cur[:, :-c, 2]
            cur[:, :-c, 0] = cur[:, c:, 0]

        # 4) 주사선 + 비네트 (결합 마스크 1회 곱)
        h, w = cur.shape[:2]
        cur *= self._combined_mask(h, w)

        out = np.clip(cur, 0, 255).astype(np.uint8)
        self.ms = (time.perf_counter() - t0) * 1000.0
        return out


def selftest():
    # 크기: 640x480 → x2 → 1280x960
    fx = RetroEffect()
    out = fx.apply(np.zeros((480, 640, 3), np.uint8))
    assert out.shape == (960, 1280, 3), '업스케일 크기 오류: %s' % (out.shape,)

    # 잔상: 밝은 사각형이 사라진 뒤에도 흔적이 남고, 프레임마다 감쇠
    fx = RetroEffect(ghost=0.7, scanline=0.0, vignette=0.0, chroma=0, upscale=1.0)
    f1 = np.zeros((100, 100, 3), np.uint8)
    f1[40:60, 20:40] = 255
    fx.apply(f1)
    black = np.zeros((100, 100, 3), np.uint8)
    out2 = fx.apply(black)
    assert out2[50, 30].max() >= int(255 * 0.7) - 2, '잔상 없음'
    out3 = fx.apply(black)
    assert out3[50, 30].max() < out2[50, 30].max(), '잔상이 감쇠하지 않음'

    # 주사선: 균일 회색 입력 → 행 밝기가 주기적으로 교차
    fx = RetroEffect(ghost=0.0, scanline=0.5, vignette=0.0, chroma=0, upscale=2.0)
    o = fx.apply(np.full((100, 100, 3), 200, np.uint8))
    assert o[100, 100, 0] > o[101, 100, 0], '주사선 없음'

    # 비네트: 모서리 < 중앙
    fx = RetroEffect(ghost=0.0, scanline=0.0, vignette=0.5, chroma=0, upscale=1.0)
    o = fx.apply(np.full((200, 200, 3), 200, np.uint8))
    assert o[5, 5, 0] < o[100, 100, 0], '비네트 없음'

    # 잔상 버퍼 오염 방지: upscale=1 에서 주사선이 잔상에 누적되면 안 됨
    fx = RetroEffect(ghost=0.9, scanline=0.5, vignette=0.0, chroma=0, upscale=1.0)
    g = np.full((100, 100, 3), 200, np.uint8)
    fx.apply(g)
    o = fx.apply(g)
    assert o[1, 50, 0] >= int(200 * 0.5) - 2, '주사선이 잔상 버퍼에 누적됨'

    # 입력 크기 변경 → 잔상 리셋, 크래시 없음
    fx = RetroEffect()
    fx.apply(np.zeros((480, 640, 3), np.uint8))
    fx.apply(np.zeros((240, 320, 3), np.uint8))

    # 타이밍 실측 (이 기기 기준 — 로봇에서 실행하면 Pi 실측이 나온다)
    fx = RetroEffect()
    rng = np.random.default_rng(0)
    frame = rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)
    for _ in range(5):
        fx.apply(frame)
    ts = []
    for _ in range(30):
        fx.apply(frame)
        ts.append(fx.ms)
    ts.sort()
    print('selftest OK — 640x480→1280x960 효과: median %.1fms / p90 %.1fms (이 기기 기준)'
          % (ts[15], ts[27]))


if __name__ == '__main__':
    selftest()

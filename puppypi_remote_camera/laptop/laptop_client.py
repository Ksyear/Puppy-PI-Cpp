#!/usr/bin/env python3
"""macOS Tkinter client for PuppyPi live video, teleoperation and local recording."""

import argparse
import collections
import json
import math
import os
import socket
import struct
import sys
import threading
import time
import uuid
from pathlib import Path
from typing import Callable, Tuple

import cv2
import numpy as np
import tkinter as tk
import yaml
from PIL import Image, ImageTk
from tkinter import messagebox, simpledialog, ttk

from video_recorder import RecordingResult, VideoRecorder


FRAME_HEADER = struct.Struct("!4sIQQHHH")
FRAME_MAGIC = b"PRC1"
MACOS_PHYSICAL_KEYS = {
    0: "a",
    1: "s",
    2: "d",
    12: "q",
    13: "w",
    14: "e",
    15: "r",
    49: "space",
}


class ControlClient:
    """Periodic UDP sender with sequence/timestamp fields and robot acknowledgements."""

    def __init__(self, robot_ip: str, config: dict):
        self._robot_ip = socket.gethostbyname(robot_ip)
        self._port = int(config["control_port"])
        self._protocol = int(config.get("protocol_version", 1))
        self._send_rate = float(config.get("control_send_rate_hz", 20.0))
        self._stop_repetitions = int(config.get("stop_repetitions", 5))
        if not 1 <= self._port <= 65535 or self._send_rate < 10.0:
            raise ValueError("조종 포트 또는 송신 주기 설정 오류")

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.connect((self._robot_ip, self._port))
        self._socket.settimeout(0.2)
        self._client_id = str(uuid.uuid4())
        self._sequence = 0
        self._send_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._running = threading.Event()
        self._wake = threading.Event()
        self._sender_thread = None
        self._ack_thread = None

        self._x = 0.0
        self._yaw_rate = 0.0
        self._movement_enabled = False
        self._emergency_requested = False
        self._clear_emergency_burst = 0
        self._stop_burst = self._stop_repetitions
        self._last_ack_at = 0.0
        self._last_ack_accepted = False
        self._last_ack_reason = "아직 응답 없음"
        self._robot_emergency = False

    def start(self):
        if self._running.is_set():
            return
        self._running.set()
        self._sender_thread = threading.Thread(
            target=self._sender_loop,
            name="control-sender",
            daemon=True,
        )
        self._ack_thread = threading.Thread(
            target=self._ack_loop,
            name="control-ack",
            daemon=True,
        )
        self._sender_thread.start()
        self._ack_thread.start()

    def set_motion(self, x: float, yaw_rate: float, enabled: bool):
        if not math.isfinite(x) or not math.isfinite(yaw_rate):
            raise ValueError("속도 명령은 유한해야 합니다")
        with self._state_lock:
            changed = (
                self._x != float(x)
                or self._yaw_rate != float(yaw_rate)
                or self._movement_enabled != bool(enabled)
            )
            self._x = float(x)
            self._yaw_rate = float(yaw_rate)
            self._movement_enabled = bool(enabled)
        if changed:
            self._wake.set()

    def emergency_stop(self):
        with self._state_lock:
            self._emergency_requested = True
            self._movement_enabled = False
            self._x = 0.0
            self._yaw_rate = 0.0
            self._stop_burst = self._stop_repetitions
        self._wake.set()

    def clear_emergency(self):
        with self._state_lock:
            self._emergency_requested = False
            self._movement_enabled = False
            self._x = 0.0
            self._yaw_rate = 0.0
            self._clear_emergency_burst = self._stop_repetitions
            self._stop_burst = self._stop_repetitions
        self._wake.set()

    def safety_stop_burst(self):
        with self._state_lock:
            self._movement_enabled = False
            self._x = 0.0
            self._yaw_rate = 0.0
            self._stop_burst = max(self._stop_burst, self._stop_repetitions)
        self._wake.set()

    def immediate_stop(self, repetitions: int = 2):
        with self._state_lock:
            self._x = 0.0
            self._yaw_rate = 0.0
        for _ in range(max(1, repetitions)):
            self._send("command", 0.0, 0.0)

    @property
    def control_alive(self) -> bool:
        with self._state_lock:
            return (
                self._last_ack_accepted
                and time.monotonic() - self._last_ack_at < 1.0
            )

    @property
    def ack_reason(self) -> str:
        with self._state_lock:
            return self._last_ack_reason

    @property
    def robot_emergency(self) -> bool:
        with self._state_lock:
            return self._emergency_requested or self._robot_emergency

    def shutdown(self):
        if not self._running.is_set():
            return
        with self._state_lock:
            self._movement_enabled = False
            self._x = 0.0
            self._yaw_rate = 0.0
        for _ in range(self._stop_repetitions):
            self._send("command", 0.0, 0.0)
            time.sleep(0.02)
        self._send("disconnect", 0.0, 0.0)
        self._running.clear()
        self._wake.set()
        try:
            self._socket.close()
        except OSError:
            pass
        for thread in (self._sender_thread, self._ack_thread):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=1.0)

    def _sender_loop(self):
        period = 1.0 / self._send_rate
        while self._running.is_set():
            self._wake.wait(timeout=period)
            self._wake.clear()
            with self._state_lock:
                if self._emergency_requested:
                    action = "emergency_stop"
                    x = 0.0
                    yaw_rate = 0.0
                elif self._clear_emergency_burst > 0:
                    self._clear_emergency_burst -= 1
                    action = "clear_emergency"
                    x = 0.0
                    yaw_rate = 0.0
                else:
                    action = "command"
                    if self._stop_burst > 0:
                        self._stop_burst -= 1
                        x = 0.0
                        yaw_rate = 0.0
                    elif self._movement_enabled:
                        x = self._x
                        yaw_rate = self._yaw_rate
                    else:
                        x = 0.0
                        yaw_rate = 0.0
            self._send(action, x, yaw_rate)

    def _send(self, action: str, x: float, yaw_rate: float):
        with self._send_lock:
            packet = {
                "protocol": self._protocol,
                "type": action,
                "client_id": self._client_id,
                "sequence": self._sequence,
                "timestamp": time.time(),
            }
            self._sequence += 1
            if action == "command":
                packet["x"] = float(x)
                packet["yaw_rate"] = float(yaw_rate)
            try:
                encoded = json.dumps(
                    packet,
                    separators=(",", ":"),
                    allow_nan=False,
                ).encode("utf-8")
                self._socket.send(encoded)
            except (OSError, ValueError):
                pass

    def _ack_loop(self):
        while self._running.is_set():
            try:
                data = self._socket.recv(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                payload = json.loads(data.decode("utf-8"))
                if (
                    payload.get("protocol") != self._protocol
                    or payload.get("type") != "ack"
                    or type(payload.get("accepted")) is not bool
                    or type(payload.get("emergency_stop")) is not bool
                ):
                    continue
                reason = str(payload.get("reason", ""))[:200]
                with self._state_lock:
                    self._last_ack_at = time.monotonic()
                    self._last_ack_accepted = payload["accepted"]
                    self._last_ack_reason = reason
                    self._robot_emergency = payload["emergency_stop"]
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue


class VideoReceiver:
    """Reconnects to the robot and exposes only the latest decoded frame."""

    def __init__(
        self,
        robot_ip: str,
        config: dict,
        frame_handler: Callable,
        disconnect_handler: Callable[[], None],
    ):
        self._robot_ip = robot_ip
        self._port = int(config["video_port"])
        self._connect_timeout = float(config.get("connect_timeout_seconds", 3.0))
        self._frame_timeout = float(config.get("video_frame_timeout_seconds", 1.0))
        self._reconnect_delay = float(config.get("reconnect_delay_seconds", 1.0))
        self._max_jpeg_bytes = int(config.get("max_jpeg_bytes", 20 * 1024 * 1024))
        self._receive_buffer = int(config.get("tcp_receive_buffer_bytes", 131072))
        self._frame_handler = frame_handler
        self._disconnect_handler = disconnect_handler
        if not 16384 <= self._receive_buffer <= 4 * 1024 * 1024:
            raise ValueError("tcp_receive_buffer_bytes 범위 오류")

        self._running = threading.Event()
        self._lock = threading.Lock()
        self._socket = None
        self._thread = None
        self._connected = False
        self._status_message = "연결 대기"
        self._latest = None
        self._receive_times = collections.deque()
        self._first_frame_logged = False

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def status_message(self) -> str:
        with self._lock:
            return self._status_message

    def start(self):
        if self._running.is_set():
            return
        self._running.set()
        self._thread = threading.Thread(
            target=self._run,
            name="video-receiver",
            daemon=True,
        )
        self._thread.start()

    def snapshot(self):
        with self._lock:
            return self._latest

    def has_fresh_frame(self, max_age_seconds: float = 0.5) -> bool:
        with self._lock:
            return (
                self._connected
                and self._latest is not None
                and time.monotonic() - self._latest[6] <= max_age_seconds
            )

    def shutdown(self):
        self._running.clear()
        with self._lock:
            current_socket = self._socket
            self._socket = None
        if current_socket is not None:
            try:
                current_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                current_socket.close()
            except OSError:
                pass
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)

    def _run(self):
        while self._running.is_set():
            current_socket = None
            try:
                current_socket = socket.create_connection(
                    (self._robot_ip, self._port),
                    timeout=self._connect_timeout,
                )
                current_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                current_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                current_socket.setsockopt(
                    socket.SOL_SOCKET,
                    socket.SO_RCVBUF,
                    self._receive_buffer,
                )
                current_socket.settimeout(self._frame_timeout)
                with self._lock:
                    self._socket = current_socket
                    self._connected = True
                    self._status_message = "영상 연결됨"
                    self._receive_times.clear()
                    self._first_frame_logged = False
                self._receive_frames(current_socket)
            except (OSError, ValueError, TimeoutError) as exc:
                self._set_disconnected(str(exc))
            finally:
                if current_socket is not None:
                    try:
                        current_socket.close()
                    except OSError:
                        pass
                with self._lock:
                    if self._socket is current_socket:
                        self._socket = None
                self._set_disconnected("영상 연결 끊김")
            deadline = time.monotonic() + self._reconnect_delay
            while self._running.is_set() and time.monotonic() < deadline:
                time.sleep(0.05)

    def _receive_frames(self, current_socket: socket.socket):
        last_sequence = -1
        while self._running.is_set():
            header = self._recv_exact(current_socket, FRAME_HEADER.size)
            magic, jpeg_size, sequence, timestamp_ns, width, height, fps_x100 = (
                FRAME_HEADER.unpack(header)
            )
            if magic != FRAME_MAGIC:
                raise ValueError("영상 프로토콜 magic 불일치")
            if not 1 <= jpeg_size <= self._max_jpeg_bytes:
                raise ValueError("JPEG 프레임 크기 범위 오류")
            if sequence <= last_sequence:
                raise ValueError("영상 sequence 역전")
            if width < 1 or height < 1:
                raise ValueError("영상 해상도 오류")
            jpeg = self._recv_exact(current_socket, jpeg_size)
            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None or frame.size == 0:
                raise ValueError("JPEG 디코딩 실패")
            decoded_height, decoded_width = frame.shape[:2]
            if decoded_width != width or decoded_height != height:
                raise ValueError("헤더와 JPEG 해상도 불일치")

            received_at = time.monotonic()
            with self._lock:
                self._receive_times.append(received_at)
                cutoff = received_at - 2.0
                while self._receive_times and self._receive_times[0] < cutoff:
                    self._receive_times.popleft()
                if len(self._receive_times) >= 2:
                    receive_fps = (len(self._receive_times) - 1) / (
                        self._receive_times[-1] - self._receive_times[0]
                    )
                else:
                    receive_fps = 0.0
                source_fps = fps_x100 / 100.0
                self._latest = (
                    sequence,
                    frame,
                    width,
                    height,
                    source_fps,
                    receive_fps,
                    received_at,
                    timestamp_ns,
                )
                self._status_message = "영상 연결됨"
                log_first_frame = not self._first_frame_logged
                self._first_frame_logged = True
            last_sequence = sequence
            if log_first_frame:
                print(
                    "영상 첫 프레임 수신: %dx%d, 카메라 설정 %.2f fps"
                    % (width, height, source_fps),
                    flush=True,
                )
            self._frame_handler(frame, received_at)

    @staticmethod
    def _recv_exact(current_socket: socket.socket, size: int) -> bytes:
        buffer = bytearray(size)
        view = memoryview(buffer)
        received = 0
        while received < size:
            count = current_socket.recv_into(view[received:])
            if count == 0:
                raise ConnectionError("상대가 TCP 연결을 닫음")
            received += count
        return bytes(buffer)

    def _set_disconnected(self, message: str):
        notify = False
        with self._lock:
            if self._connected:
                notify = True
            self._connected = False
            self._status_message = message[:200]
        if notify:
            self._disconnect_handler()


class LaptopApplication:
    MOTION_KEYS = {"w", "s", "a", "d"}

    def __init__(self, root: tk.Tk, robot_ip: str, config: dict):
        self.root = root
        self.config = config
        self.robot_ip = robot_ip
        self._closing = False
        self._window_focused = False
        self._pressed_motion = set()
        self._pressed_actions = set()
        self._motion_release_jobs = {}
        self._last_display_sequence = -1
        self._last_video_connected = False
        self._first_display_logged = False
        self._photo = None

        recording = config["recording"]
        self.recorder = VideoRecorder(
            recording["directory"],
            recording.get("default_fps", 30.0),
            recording.get("minimum_valid_bytes", 1024),
            recording.get("max_fill_gap_seconds", 2.0),
        )
        self.controller = ControlClient(robot_ip, config["network"])
        self.receiver = VideoReceiver(
            robot_ip,
            config["network"],
            self._handle_received_frame,
            self.controller.safety_stop_burst,
        )

        self._build_gui()
        self._bind_keys()
        self.controller.start()
        self.receiver.start()
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)
        self.root.after(30, self._update_gui)
        self.root.after(150, self._request_initial_focus)

    def _build_gui(self):
        self.root.title("PuppyPi Remote Camera")
        self.root.geometry("1120x820")
        self.root.minsize(850, 680)

        main = ttk.Frame(self.root, padding=8)
        main.pack(fill=tk.BOTH, expand=True)

        self.video_label = tk.Label(
            main,
            bg="black",
            fg="white",
            text="PuppyPi 영상 연결 대기",
            anchor=tk.CENTER,
        )
        self.video_label.pack(fill=tk.BOTH, expand=True)
        self.video_label.bind("<Button-1>", self._take_keyboard_focus, add="+")

        status_frame = ttk.LabelFrame(main, text="상태", padding=8)
        status_frame.pack(fill=tk.X, pady=(8, 0))
        self.status_vars = {
            "connection": tk.StringVar(value="연결 대기"),
            "resolution": tk.StringVar(value="-"),
            "fps": tk.StringVar(value="-"),
            "keyboard": tk.StringVar(value="포커스 대기 / 누른 이동 키 없음"),
            "command": tk.StringVar(value="정지 x=0.00, y=0.00, yaw=0.000"),
            "recording": tk.StringVar(value="녹화 안 함"),
            "duration": tk.StringVar(value="0.0초"),
            "path": tk.StringVar(value=""),
            "emergency": tk.StringVar(value="해제"),
        }
        labels = (
            ("로봇 연결", "connection"),
            ("실제 해상도", "resolution"),
            ("실제 수신 FPS", "fps"),
            ("키보드 입력", "keyboard"),
            ("현재 이동 명령", "command"),
            ("녹화 상태", "recording"),
            ("녹화 시간", "duration"),
            ("저장 파일", "path"),
            ("비상정지", "emergency"),
        )
        for row, (title, key) in enumerate(labels):
            ttk.Label(status_frame, text=title, width=16).grid(
                row=row,
                column=0,
                sticky=tk.W,
                pady=1,
            )
            ttk.Label(status_frame, textvariable=self.status_vars[key]).grid(
                row=row,
                column=1,
                sticky=tk.W,
                pady=1,
            )
        status_frame.columnconfigure(1, weight=1)

        controls = ttk.Frame(main)
        controls.pack(fill=tk.X, pady=(8, 0))
        self.record_button = ttk.Button(
            controls,
            text="녹화 시작 (R)",
            command=self.toggle_recording,
        )
        self.record_button.pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(
            controls,
            text="비상정지 (E)",
            command=self.activate_emergency,
        ).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(
            controls,
            text="비상정지 해제",
            command=self.clear_emergency,
        ).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(
            controls,
            text="정지 (Space)",
            command=self.stop_motion,
        ).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(
            controls,
            text="종료 (Q)",
            command=self.shutdown,
        ).pack(side=tk.RIGHT)

        ttk.Label(
            main,
            text=(
                "키를 누르는 동안만 이동: W 전진 / S 후진 / A 좌회전 / "
                "D 우회전 / Space 정지 / E 비상정지 / R 녹화 / Q 종료"
            ),
        ).pack(fill=tk.X, pady=(6, 0))

    def _bind_keys(self):
        self.root.bind_all("<KeyPress>", self._on_key_press, add="+")
        self.root.bind_all("<KeyRelease>", self._on_key_release, add="+")
        self.root.bind("<FocusIn>", self._on_focus_in, add="+")
        self.root.bind("<FocusOut>", self._on_focus_out, add="+")

    def _request_initial_focus(self):
        if not self._closing:
            self.root.lift()
            self.root.focus_force()

    def _take_keyboard_focus(self, _event=None):
        if not self._closing:
            self.root.focus_force()
            self._window_focused = True
            self._update_keyboard_status()

    @staticmethod
    def _normalized_key(event) -> str:
        key = str(event.keysym).lower()
        known = LaptopApplication.MOTION_KEYS | {"space", "e", "r", "q"}
        if key in known:
            return key
        if sys.platform == "darwin":
            return MACOS_PHYSICAL_KEYS.get(event.keycode, key)
        return key

    def _on_key_press(self, event):
        key = self._normalized_key(event)
        self.status_vars["keyboard"].set(
            "인식: %s (keysym=%s, keycode=%s)"
            % (key.upper(), event.keysym, event.keycode)
        )
        if key in self.MOTION_KEYS:
            release_job = self._motion_release_jobs.pop(key, None)
            if release_job is not None:
                self.root.after_cancel(release_job)
            self._pressed_motion.add(key)
            self._apply_motion()
            return "break"
        if key == "space":
            self.stop_motion()
            return "break"
        if key in self._pressed_actions:
            return "break"
        self._pressed_actions.add(key)
        if key == "e":
            self.activate_emergency()
            return "break"
        if key == "r":
            self.toggle_recording()
            return "break"
        if key == "q":
            self.shutdown()
            return "break"
        return None

    def _on_key_release(self, event):
        key = self._normalized_key(event)
        self._pressed_actions.discard(key)
        if key in self.MOTION_KEYS:
            previous_job = self._motion_release_jobs.pop(key, None)
            if previous_job is not None:
                self.root.after_cancel(previous_job)
            self._motion_release_jobs[key] = self.root.after(
                40,
                lambda released_key=key: self._finish_motion_release(
                    released_key
                ),
            )
            return "break"
        return None

    def _finish_motion_release(self, key: str):
        self._motion_release_jobs.pop(key, None)
        self._pressed_motion.discard(key)
        self._apply_motion()
        if not self._pressed_motion:
            self.controller.immediate_stop(2)

    def _on_focus_in(self, _event):
        self._window_focused = True
        self._update_keyboard_status()
        self._apply_motion()

    def _on_focus_out(self, _event):
        self.root.after(20, self._verify_focus_lost)

    def _verify_focus_lost(self):
        if self._closing:
            return
        try:
            focused_widget = self.root.focus_displayof()
        except tk.TclError:
            focused_widget = None
        if focused_widget is None:
            self._window_focused = False
            for release_job in self._motion_release_jobs.values():
                self.root.after_cancel(release_job)
            self._motion_release_jobs.clear()
            self._pressed_motion.clear()
            self._pressed_actions.clear()
            self.controller.safety_stop_burst()
            self._update_keyboard_status()

    def _update_keyboard_status(self):
        pressed = "+".join(sorted(key.upper() for key in self._pressed_motion))
        self.status_vars["keyboard"].set(
            "포커스 %s / 누른 이동 키 %s"
            % ("있음" if self._window_focused else "없음", pressed or "없음")
        )

    def _motion_values(self) -> Tuple[float, float, str]:
        movement = self.config["movement"]
        if "w" in self._pressed_motion and "s" not in self._pressed_motion:
            x = float(movement["forward_cm_s"])
        elif "s" in self._pressed_motion and "w" not in self._pressed_motion:
            x = -float(movement["reverse_cm_s"])
        else:
            x = 0.0
        if "a" in self._pressed_motion and "d" not in self._pressed_motion:
            yaw = float(movement["yaw_rate_rad_s"])
        elif "d" in self._pressed_motion and "a" not in self._pressed_motion:
            yaw = -float(movement["yaw_rate_rad_s"])
        else:
            yaw = 0.0
        if x == 0.0 and yaw == 0.0:
            name = "정지"
        else:
            parts = []
            if x > 0:
                parts.append("전진")
            elif x < 0:
                parts.append("후진")
            if yaw > 0:
                parts.append("좌회전")
            elif yaw < 0:
                parts.append("우회전")
            name = "+".join(parts)
        return x, yaw, name

    def _apply_motion(self):
        x, yaw, name = self._motion_values()
        blocked = []
        if not self._window_focused:
            blocked.append("창 포커스 없음")
        if not self.receiver.has_fresh_frame():
            blocked.append("최신 영상 없음")
        if not self.controller.control_alive:
            blocked.append("제어 ACK 없음")
        if self.controller.robot_emergency:
            blocked.append("비상정지")
        enabled = not blocked
        if not enabled:
            x = 0.0
            yaw = 0.0
            if self._pressed_motion:
                name = "차단(%s)" % ", ".join(blocked)
            else:
                name = "정지"
        self.controller.set_motion(x, yaw, enabled)
        self._update_keyboard_status()
        self.status_vars["command"].set(
            "%s x=%+.2f, y=0.00, yaw=%+.3f" % (name, x, yaw)
        )

    def stop_motion(self):
        self._pressed_motion.clear()
        self.controller.set_motion(0.0, 0.0, self.receiver.connected)
        self.controller.immediate_stop(3)
        self.status_vars["command"].set("정지 x=0.00, y=0.00, yaw=0.000")

    def activate_emergency(self):
        self._pressed_motion.clear()
        self.controller.emergency_stop()
        self.status_vars["emergency"].set("활성")
        self.status_vars["command"].set("비상정지 x=0.00, y=0.00, yaw=0.000")

    def clear_emergency(self):
        self._pressed_motion.clear()
        self.controller.clear_emergency()
        self.status_vars["emergency"].set("해제 요청 중")
        self.status_vars["command"].set("정지 x=0.00, y=0.00, yaw=0.000")

    def toggle_recording(self):
        if self.recorder.is_recording:
            self._finish_recording()
            return
        snapshot = self.receiver.snapshot()
        if snapshot is None or not self.receiver.has_fresh_frame():
            messagebox.showerror(
                "녹화 시작 실패",
                "현재 수신 중인 영상 프레임이 없습니다.",
            )
            return
        _, frame, width, height, source_fps, receive_fps, _, _ = snapshot
        fps = source_fps or receive_fps or float(
            self.config["recording"].get("default_fps", 30.0)
        )
        try:
            path, fallback = self.recorder.start(frame, fps)
        except (OSError, ValueError, RuntimeError) as exc:
            messagebox.showerror("녹화 시작 실패", str(exc))
            return
        self.record_button.configure(text="녹화 종료 (R)")
        self.status_vars["recording"].set(
            "녹화 중" if not fallback else "녹화 중 - %s" % fallback
        )
        self.status_vars["duration"].set("0.0초")
        self.status_vars["path"].set(path)
        print("녹화 시작: %s (%dx%d, %.3f fps)" % (path, width, height, fps))
        if fallback:
            print(fallback, file=sys.stderr)

    def _finish_recording(self):
        result = self.recorder.stop()
        self.record_button.configure(text="녹화 시작 (R)")
        self.status_vars["recording"].set(
            "녹화 완료" if result.success else "녹화 실패"
        )
        self.status_vars["duration"].set("%.1f초" % result.duration_seconds)
        if result.path:
            self.status_vars["path"].set(result.path)
        self._print_recording_result(result)
        if not result.success:
            messagebox.showerror("녹화 검증 실패", result.message)

    @staticmethod
    def _print_recording_result(result: RecordingResult):
        stream = sys.stdout if result.success else sys.stderr
        print(
            "%s: 경로=%s, 해상도=%dx%d, FPS=%.3f, 녹화 시간=%.3f초, "
            "기록 프레임=%d, 검사=%s"
            % (
                "녹화 성공" if result.success else "녹화 실패",
                result.path or "-",
                result.width,
                result.height,
                result.fps,
                result.duration_seconds,
                result.frames_written,
                result.message,
            ),
            file=stream,
        )

    def _handle_received_frame(self, frame, received_at: float):
        if self.recorder.is_recording:
            self.recorder.add_frame(frame, received_at)

    def _update_gui(self):
        if self._closing:
            return
        video_connected = self.receiver.connected
        if self._last_video_connected and not video_connected:
            self._pressed_motion.clear()
            self.controller.safety_stop_burst()
        self._last_video_connected = video_connected
        snapshot = self.receiver.snapshot()
        if snapshot is not None:
            (
                sequence,
                frame,
                width,
                height,
                source_fps,
                receive_fps,
                received_at,
                _,
            ) = snapshot
            self.status_vars["resolution"].set("%dx%d" % (width, height))
            self.status_vars["fps"].set(
                "%.2f fps (카메라 설정 %.2f)" % (receive_fps, source_fps)
            )
            if sequence != self._last_display_sequence:
                self._last_display_sequence = sequence
                self._show_frame(frame)
            if time.monotonic() - received_at > 0.5:
                self.controller.safety_stop_burst()

        if self.receiver.has_fresh_frame():
            video_state = "영상 정상"
        elif video_connected:
            video_state = "TCP 연결됨 / 정상 영상 프레임 없음"
        else:
            video_state = self.receiver.status_message
        control_state = (
            "제어 응답 정상"
            if self.controller.control_alive
            else "제어 응답 없음: %s" % self.controller.ack_reason
        )
        self.status_vars["connection"].set("%s / %s" % (video_state, control_state))
        self.status_vars["emergency"].set(
            "활성" if self.controller.robot_emergency else "해제"
        )
        if self.recorder.is_recording:
            self.status_vars["duration"].set(
                "%.1f초" % self.recorder.elapsed_seconds
            )
            if self.recorder.runtime_error:
                self._finish_recording()

        self._apply_motion()
        self.root.after(30, self._update_gui)

    def _show_frame(self, frame):
        max_width = int(self.config["display"].get("max_width", 1024))
        max_height = int(self.config["display"].get("max_height", 576))
        height, width = frame.shape[:2]
        scale = min(max_width / width, max_height / height, 1.0)
        shown_width = max(1, int(round(width * scale)))
        shown_height = max(1, int(round(height * scale)))
        if shown_width != width or shown_height != height:
            display_frame = cv2.resize(
                frame,
                (shown_width, shown_height),
                interpolation=cv2.INTER_AREA,
            )
        else:
            display_frame = frame
        rgb = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
        self._photo = ImageTk.PhotoImage(
            Image.fromarray(rgb),
            master=self.root,
        )
        self.video_label.configure(image=self._photo, text="")
        if not self._first_display_logged:
            self._first_display_logged = True
            print(
                "GUI 첫 프레임 표시 완료: %dx%d -> %dx%d"
                % (width, height, shown_width, shown_height),
                flush=True,
            )

    def shutdown(self):
        if self._closing:
            return
        self._closing = True
        for release_job in self._motion_release_jobs.values():
            self.root.after_cancel(release_job)
        self._motion_release_jobs.clear()
        self._pressed_motion.clear()
        self.controller.safety_stop_burst()
        if self.recorder.is_recording:
            result = self.recorder.stop()
            self._print_recording_result(result)
        self.controller.shutdown()
        self.receiver.shutdown()
        self.root.destroy()


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise ValueError("laptop_config.yaml 최상위 값은 객체여야 합니다")
    for section in ("network", "movement", "recording", "display"):
        if not isinstance(config.get(section), dict):
            raise ValueError("설정 섹션 누락: %s" % section)
    return config


def test_video_once(robot_ip: str, config: dict) -> int:
    network = config["network"]
    port = int(network["video_port"])
    timeout = float(network.get("connect_timeout_seconds", 3.0))
    max_jpeg_bytes = int(network.get("max_jpeg_bytes", 20 * 1024 * 1024))

    with socket.create_connection((robot_ip, port), timeout=timeout) as sock:
        sock.settimeout(max(5.0, timeout))
        header = VideoReceiver._recv_exact(sock, FRAME_HEADER.size)
        magic, size, sequence, _, width, height, fps_x100 = (
            FRAME_HEADER.unpack(header)
        )
        if magic != FRAME_MAGIC:
            raise ValueError("영상 프로토콜 magic 불일치: %r" % (magic,))
        if not 1 <= size <= max_jpeg_bytes:
            raise ValueError("JPEG 크기 범위 오류: %d bytes" % size)
        jpeg = VideoReceiver._recv_exact(sock, size)

    frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None or frame.size == 0:
        raise ValueError("수신 JPEG를 디코딩할 수 없습니다")
    decoded_height, decoded_width = frame.shape[:2]
    if decoded_width != width or decoded_height != height:
        raise ValueError(
            "헤더와 JPEG 해상도 불일치: %dx%d / %dx%d"
            % (width, height, decoded_width, decoded_height)
        )

    output = Path("/tmp/puppypi_test.jpg")
    output.write_bytes(jpeg)
    print("sequence=%d" % sequence)
    print("resolution=%dx%d" % (width, height))
    print("camera_fps=%.2f" % (fps_x100 / 100.0))
    print("jpeg_bytes=%d" % size)
    print("saved=%s" % output)
    return 0


def build_argument_parser() -> argparse.ArgumentParser:
    default_config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "laptop_config.yaml")
    )
    parser = argparse.ArgumentParser(
        description="PuppyPi 실시간 영상/키보드 조종/노트북 녹화 GUI"
    )
    parser.add_argument("--robot-ip", help="PuppyPi IPv4 주소 또는 호스트명")
    parser.add_argument(
        "--config",
        default=default_config,
        help="laptop_config.yaml 경로",
    )
    parser.add_argument(
        "--test-video",
        action="store_true",
        help="GUI 없이 영상 한 프레임을 받아 /tmp/puppypi_test.jpg로 저장",
    )
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    try:
        config = load_config(os.path.abspath(os.path.expanduser(args.config)))
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print("설정 오류: %s" % exc, file=sys.stderr)
        return 2

    robot_ip = args.robot_ip or str(config["network"].get("robot_ip", "")).strip()
    if args.test_video:
        if not robot_ip:
            print("--test-video에는 --robot-ip가 필요합니다.", file=sys.stderr)
            return 2
        try:
            return test_video_once(robot_ip.strip(), config)
        except (OSError, ValueError, TimeoutError) as exc:
            print("영상 한 프레임 시험 실패: %s" % exc, file=sys.stderr)
            return 1

    root = tk.Tk()
    if not robot_ip:
        root.withdraw()
        robot_ip = simpledialog.askstring(
            "PuppyPi 연결",
            "PuppyPi IP 주소를 입력하십시오:",
            parent=root,
        )
        root.deiconify()
    if not robot_ip:
        root.destroy()
        print("PuppyPi IP가 없어 종료합니다.", file=sys.stderr)
        return 2

    try:
        LaptopApplication(root, robot_ip.strip(), config)
    except (OSError, ValueError, RuntimeError) as exc:
        root.destroy()
        print("클라이언트 시작 실패: %s" % exc, file=sys.stderr)
        return 1
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

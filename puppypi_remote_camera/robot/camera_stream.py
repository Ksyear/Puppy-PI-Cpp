#!/usr/bin/env python3
"""V4L2 camera discovery, capability probing and latest-frame TCP streaming."""

import logging
import os
import re
import socket
import struct
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import cv2


LOGGER = logging.getLogger(__name__)
FRAME_HEADER = struct.Struct("!4sIQQHHH")
FRAME_MAGIC = b"PRC1"


@dataclass(frozen=True)
class CameraMode:
    pixel_format: str
    width: int
    height: int
    fps: float


@dataclass(frozen=True)
class CameraSelection:
    device_path: str
    description: str
    modes: Tuple[CameraMode, ...]


def _run_v4l2(arguments: List[str]) -> str:
    try:
        result = subprocess.run(
            ["v4l2-ctl"] + arguments,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=10,
        )
    except FileNotFoundError:
        raise RuntimeError(
            "v4l2-ctl이 없습니다. sudo apt install v4l-utils를 실행하십시오"
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        output = getattr(exc, "stdout", "") or ""
        raise RuntimeError("v4l2-ctl 실패: %s" % output.strip())
    return result.stdout


def parse_device_list(output: str) -> List[Tuple[str, List[str]]]:
    devices = []
    current_name = None
    current_nodes = []  # type: List[str]
    for raw_line in output.splitlines():
        line = raw_line.rstrip()
        if line and not line[0].isspace() and line.endswith(":"):
            if current_name is not None:
                devices.append((current_name, current_nodes))
            current_name = line[:-1].strip()
            current_nodes = []
        elif current_name is not None:
            node = line.strip()
            if re.fullmatch(r"/dev/video\d+", node):
                current_nodes.append(node)
    if current_name is not None:
        devices.append((current_name, current_nodes))
    return devices


def parse_formats(output: str) -> Tuple[CameraMode, ...]:
    modes = []  # type: List[CameraMode]
    current_format = None
    for raw_line in output.splitlines():
        format_match = re.search(r"\[\d+\]:\s+'([^']+)'", raw_line)
        if format_match:
            current_format = format_match.group(1).upper()
            continue
        size_match = re.search(r"Size:\s+\w+\s+(\d+)x(\d+)", raw_line)
        if size_match and current_format:
            modes.append(
                CameraMode(
                    current_format,
                    int(size_match.group(1)),
                    int(size_match.group(2)),
                    0.0,
                )
            )
            continue
        fps_matches = re.findall(r"\(([\d.]+)\s+fps\)", raw_line)
        if fps_matches and modes:
            best_fps = max(float(value) for value in fps_matches)
            previous = modes[-1]
            modes[-1] = CameraMode(
                previous.pixel_format,
                previous.width,
                previous.height,
                max(previous.fps, best_fps),
            )
    return tuple(modes)


def _by_id_aliases() -> Dict[str, List[str]]:
    aliases = {}  # type: Dict[str, List[str]]
    by_id = Path("/dev/v4l/by-id")
    if not by_id.is_dir():
        return aliases
    for entry in sorted(by_id.iterdir()):
        try:
            target = os.path.realpath(str(entry))
        except OSError:
            continue
        aliases.setdefault(target, []).append(str(entry))
    return aliases


def discover_camera(name_contains: str) -> CameraSelection:
    """Select only a named V4L2 capture device; never assume /dev/video0."""
    needle = name_contains.strip().casefold()
    if not needle:
        raise ValueError("camera.name_contains가 비어 있습니다")

    listed = parse_device_list(_run_v4l2(["--list-devices"]))
    aliases = _by_id_aliases()
    diagnostics = []
    matches = []

    for description, nodes in listed:
        for node in nodes:
            node_aliases = aliases.get(os.path.realpath(node), [])
            searchable = " ".join([description, node] + node_aliases).casefold()
            diagnostics.append(
                "%s -> %s (%s)"
                % (description, node, ", ".join(node_aliases))
            )
            if needle not in searchable:
                continue
            try:
                formats_text = _run_v4l2(["--device", node, "--list-formats-ext"])
                modes = parse_formats(formats_text)
                capabilities = _run_v4l2(["--device", node, "--all"])
            except RuntimeError as exc:
                LOGGER.warning("후보 장치 조회 실패 %s: %s", node, exc)
                continue
            if not modes or "Video Capture" not in capabilities:
                continue
            open_path = node_aliases[0] if node_aliases else node
            matches.append(CameraSelection(open_path, description, modes))

    if not matches:
        detail = "\n".join(diagnostics) if diagnostics else "(V4L2 장치 없음)"
        raise RuntimeError(
            "'%s'와 일치하는 영상 캡처 장치를 찾지 못했습니다.\n%s"
            % (name_contains, detail)
        )
    if len(matches) > 1:
        LOGGER.warning(
            "이름과 일치하는 캡처 노드가 %d개입니다. "
            "첫 번째 유효 노드를 사용합니다: %s",
            len(matches),
            matches[0].device_path,
        )
    return matches[0]


def select_camera_mode(
    modes: Tuple[CameraMode, ...],
    preferred_width: int,
    preferred_height: int,
    preferred_fps: float,
) -> CameraMode:
    if not modes:
        raise ValueError("지원 영상 모드가 없습니다")

    def score(mode):
        is_mjpeg = mode.pixel_format in {"MJPG", "JPEG"}
        exact_size = mode.width == preferred_width and mode.height == preferred_height
        enough_fps = mode.fps >= preferred_fps - 0.01
        tier = (
            5 if is_mjpeg and exact_size and enough_fps
            else 4 if exact_size and enough_fps
            else 3 if is_mjpeg and exact_size
            else 2 if exact_size
            else 1 if is_mjpeg
            else 0
        )
        size_error = (
            abs(mode.width - preferred_width)
            + abs(mode.height - preferred_height)
        )
        return tier, -size_error, mode.fps, mode.width * mode.height

    return max(modes, key=score)


class CameraStreamer:
    """Captures continuously and sends only the newest JPEG to one TCP client."""

    def __init__(
        self,
        config: dict,
        on_client_connected: Callable[[str], None],
        on_client_disconnected: Callable[[str], None],
    ):
        self._camera_name = str(config["camera"]["name_contains"])
        self._preferred_width = int(config["camera"].get("preferred_width", 1920))
        self._preferred_height = int(config["camera"].get("preferred_height", 1080))
        self._preferred_fps = float(config["camera"].get("preferred_fps", 30.0))
        self._jpeg_quality = int(config["camera"].get("jpeg_quality", 85))
        self._send_buffer = int(
            config["camera"].get("tcp_send_buffer_bytes", 131072)
        )
        self._bind_address = str(config.get("bind_address", "0.0.0.0"))
        self._video_port = int(config["video_port"])
        self._frame_timeout = float(config["camera"].get("frame_timeout_seconds", 1.0))
        self._on_connected = on_client_connected
        self._on_disconnected = on_client_disconnected

        if not 1 <= self._video_port <= 65535:
            raise ValueError("video_port 범위 오류")
        if not 1 <= self._jpeg_quality <= 100:
            raise ValueError("jpeg_quality 범위는 1~100입니다")
        if not 16384 <= self._send_buffer <= 4 * 1024 * 1024:
            raise ValueError("tcp_send_buffer_bytes 범위 오류")
        if self._frame_timeout <= 0:
            raise ValueError("frame_timeout_seconds는 0보다 커야 합니다")

        self._running = threading.Event()
        self._condition = threading.Condition()
        self._latest = None
        self._sequence = 0
        self._capture = None  # type: Optional[cv2.VideoCapture]
        self._listener = None  # type: Optional[socket.socket]
        self._capture_thread = None  # type: Optional[threading.Thread]
        self._server_thread = None  # type: Optional[threading.Thread]
        self._actual_mode = None  # type: Optional[CameraMode]
        self._selection = None  # type: Optional[CameraSelection]

    @property
    def selection(self) -> Optional[CameraSelection]:
        return self._selection

    @property
    def actual_mode(self) -> Optional[CameraMode]:
        return self._actual_mode

    def start(self):
        if self._running.is_set():
            return
        selection = discover_camera(self._camera_name)
        selected_mode = select_camera_mode(
            selection.modes,
            self._preferred_width,
            self._preferred_height,
            self._preferred_fps,
        )
        self._selection = selection
        self._capture = self._open_capture(selection.device_path, selected_mode)

        actual_width = int(round(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
        actual_height = int(round(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        actual_fps = float(self._capture.get(cv2.CAP_PROP_FPS))
        actual_pixel_format = self._decode_fourcc(
            self._capture.get(cv2.CAP_PROP_FOURCC)
        )
        if not actual_fps or actual_fps <= 0:
            actual_fps = selected_mode.fps or self._preferred_fps
        self._actual_mode = CameraMode(
            actual_pixel_format,
            actual_width,
            actual_height,
            actual_fps,
        )

        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self._bind_address, self._video_port))
        listener.listen(1)
        listener.settimeout(0.5)
        self._listener = listener

        self._running.set()
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name="camera-capture",
            daemon=True,
        )
        self._server_thread = threading.Thread(
            target=self._server_loop,
            name="video-tcp",
            daemon=True,
        )
        self._capture_thread.start()
        self._server_thread.start()

        LOGGER.info(
            "선택 카메라: %s (%s)",
            selection.description,
            selection.device_path,
        )
        LOGGER.info(
            "선택 모드: %s %dx%d %.3ffps; OpenCV 보고값: %s %dx%d %.3ffps",
            selected_mode.pixel_format,
            selected_mode.width,
            selected_mode.height,
            selected_mode.fps,
            actual_pixel_format,
            actual_width,
            actual_height,
            actual_fps,
        )
        LOGGER.info("영상 TCP 수신 대기: %s:%d", self._bind_address, self._video_port)
        LOGGER.info("지원 모드:\n%s", self.format_supported_modes(selection.modes))

    def shutdown(self):
        self._running.clear()
        with self._condition:
            self._condition.notify_all()
        listener = self._listener
        self._listener = None
        if listener is not None:
            try:
                listener.close()
            except OSError:
                pass
        capture = self._capture
        self._capture = None
        if capture is not None:
            capture.release()
        for thread in (self._capture_thread, self._server_thread):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=2.0)

    @staticmethod
    def format_supported_modes(modes: Tuple[CameraMode, ...]) -> str:
        return "\n".join(
            "  %s %dx%d %.3f fps" % (
                mode.pixel_format,
                mode.width,
                mode.height,
                mode.fps,
            )
            for mode in modes
        )

    @staticmethod
    def _decode_fourcc(value: float) -> str:
        encoded = int(round(value))
        text = "".join(chr((encoded >> (8 * index)) & 0xFF) for index in range(4))
        text = text.rstrip("\x00").strip()
        return text or "UNKNOWN"

    def _open_capture(self, device_path: str, mode: CameraMode) -> cv2.VideoCapture:
        capture = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError("카메라 열기 실패: %s" % device_path)
        fourcc = cv2.VideoWriter_fourcc(*mode.pixel_format[:4])
        capture.set(cv2.CAP_PROP_FOURCC, fourcc)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, mode.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, mode.height)
        if mode.fps > 0:
            capture.set(cv2.CAP_PROP_FPS, mode.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _capture_loop(self):
        failed_reads = 0
        encode_options = [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
        while self._running.is_set():
            capture = self._capture
            if capture is None:
                break
            ok, frame = capture.read()
            if not ok or frame is None or frame.size == 0:
                failed_reads += 1
                if failed_reads == 1 or failed_reads % 30 == 0:
                    LOGGER.error("카메라 프레임 읽기 실패 (%d회)", failed_reads)
                time.sleep(0.02)
                continue
            failed_reads = 0
            height, width = frame.shape[:2]
            ok, encoded = cv2.imencode(".jpg", frame, encode_options)
            if not ok:
                LOGGER.error("JPEG 인코딩 실패")
                continue
            with self._condition:
                self._sequence += 1
                self._latest = (
                    self._sequence,
                    time.time_ns(),
                    width,
                    height,
                    bytes(encoded),
                )
                self._condition.notify_all()

    def _server_loop(self):
        while self._running.is_set():
            listener = self._listener
            if listener is None:
                break
            try:
                client, address = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            client_ip = address[0]
            LOGGER.info("영상 클라이언트 연결: %s:%d", address[0], address[1])
            self._on_connected(client_ip)
            try:
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                client.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                client.setsockopt(
                    socket.SOL_SOCKET,
                    socket.SO_SNDBUF,
                    self._send_buffer,
                )
                client.settimeout(2.0)
                self._stream_client(client)
            except (OSError, TimeoutError) as exc:
                LOGGER.warning("영상 전송 종료: %s", exc)
            finally:
                try:
                    client.close()
                except OSError:
                    pass
                self._on_disconnected(client_ip)

    def _stream_client(self, client: socket.socket):
        last_sequence = -1
        while self._running.is_set():
            deadline = time.monotonic() + self._frame_timeout
            with self._condition:
                while (
                    self._running.is_set()
                    and (self._latest is None or self._latest[0] == last_sequence)
                ):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError("새 카메라 프레임이 없습니다")
                    self._condition.wait(timeout=remaining)
                if not self._running.is_set():
                    return
                sequence, timestamp_ns, width, height, jpeg = self._latest
            last_sequence = sequence
            fps_x100 = int(
                round(min(655.35, max(0.0, self._actual_mode.fps)) * 100.0)
            )
            header = FRAME_HEADER.pack(
                FRAME_MAGIC,
                len(jpeg),
                sequence,
                timestamp_ns,
                width,
                height,
                fps_x100,
            )
            client.sendall(header)
            client.sendall(jpeg)

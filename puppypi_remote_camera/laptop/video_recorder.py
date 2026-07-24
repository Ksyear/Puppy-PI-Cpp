#!/usr/bin/env python3
"""Timestamp-aware, locally stored video recording with codec fallback."""

import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import cv2


@dataclass(frozen=True)
class RecordingResult:
    success: bool
    path: str
    width: int
    height: int
    fps: float
    duration_seconds: float
    frames_written: int
    message: str


class VideoRecorder:
    """Writes decoded received frames without resizing them."""

    CANDIDATES = (
        (".mp4", "mp4v", "MP4/mp4v"),
        (".mp4", "avc1", "MP4/avc1"),
        (".avi", "MJPG", "AVI/MJPEG"),
        (".avi", "XVID", "AVI/XVID"),
        (".mkv", "MJPG", "MKV/MJPEG"),
    )

    def __init__(
        self,
        output_directory: str,
        default_fps: float = 30.0,
        minimum_valid_bytes: int = 1024,
        max_fill_gap_seconds: float = 2.0,
    ):
        self._directory = Path(os.path.expanduser(output_directory)).resolve()
        self._directory.mkdir(parents=True, exist_ok=True)
        self._default_fps = float(default_fps)
        self._minimum_valid_bytes = int(minimum_valid_bytes)
        self._max_fill_gap_seconds = float(max_fill_gap_seconds)
        self._lock = threading.RLock()
        self._writer = None
        self._path = None  # type: Optional[Path]
        self._codec_label = ""
        self._fallback_reason = ""
        self._width = 0
        self._height = 0
        self._fps = 0.0
        self._started_at = 0.0
        self._next_timeline_index = 0
        self._frames_written = 0
        self._previous_frame = None
        self._runtime_error = ""

    @property
    def is_recording(self) -> bool:
        with self._lock:
            return self._writer is not None

    @property
    def current_path(self) -> str:
        with self._lock:
            return str(self._path) if self._path is not None else ""

    @property
    def fallback_reason(self) -> str:
        with self._lock:
            return self._fallback_reason

    @property
    def runtime_error(self) -> str:
        with self._lock:
            return self._runtime_error

    @property
    def elapsed_seconds(self) -> float:
        with self._lock:
            if self._writer is None:
                return 0.0
            return max(0.0, time.monotonic() - self._started_at)

    def start(self, initial_frame, source_fps: float) -> Tuple[str, str]:
        with self._lock:
            if self._writer is not None:
                raise RuntimeError("이미 녹화 중입니다")
            if initial_frame is None or initial_frame.size == 0:
                raise ValueError("녹화를 시작할 수신 프레임이 없습니다")

            height, width = initial_frame.shape[:2]
            fps = float(source_fps)
            if not 1.0 <= fps <= 120.0:
                fps = self._default_fps
            if not 1.0 <= fps <= 120.0:
                raise ValueError("녹화 FPS 설정이 올바르지 않습니다")

            base = self._unique_base(width, height)
            failures = []
            selected = None
            for extension, codec, label in self.CANDIDATES:
                path = base.with_suffix(extension)
                if path.exists():
                    failures.append("%s: 파일명 충돌" % label)
                    continue
                if not self._probe_writer(
                    path,
                    codec,
                    fps,
                    (width, height),
                    initial_frame,
                ):
                    failures.append("%s 코덱을 열거나 검증하지 못함" % label)
                    continue
                writer = cv2.VideoWriter(
                    str(path),
                    cv2.VideoWriter_fourcc(*codec),
                    fps,
                    (width, height),
                )
                if not writer.isOpened():
                    writer.release()
                    self._remove_created_file(path)
                    failures.append("%s 최종 writer 열기 실패" % label)
                    continue
                selected = (writer, path, label)
                break

            if selected is None:
                raise RuntimeError(
                    "사용 가능한 영상 writer가 없습니다: %s"
                    % "; ".join(failures)
                )

            self._writer, self._path, self._codec_label = selected
            self._fallback_reason = ""
            if self._path.suffix.lower() != ".mp4":
                mp4_failures = [
                    failure
                    for failure in failures
                    if failure.startswith("MP4/")
                ]
                self._fallback_reason = (
                    "MP4 저장 실패로 %s 사용: %s"
                    % (self._codec_label, "; ".join(mp4_failures))
                )
            self._width = width
            self._height = height
            self._fps = fps
            self._started_at = time.monotonic()
            self._next_timeline_index = 0
            self._frames_written = 0
            self._previous_frame = None
            self._runtime_error = ""
            self._write_frame_locked(initial_frame, self._started_at)
            return str(self._path), self._fallback_reason

    def add_frame(self, frame, received_at: Optional[float] = None) -> bool:
        with self._lock:
            if self._writer is None:
                return False
            if frame is None or frame.size == 0:
                self._runtime_error = "빈 수신 프레임"
                return False
            height, width = frame.shape[:2]
            if width != self._width or height != self._height:
                self._runtime_error = (
                    "녹화 중 수신 해상도 변경: %dx%d -> %dx%d; 프레임을 저장하지 않음"
                    % (self._width, self._height, width, height)
                )
                return False
            self._write_frame_locked(frame, received_at or time.monotonic())
            return True

    def stop(self) -> RecordingResult:
        with self._lock:
            if self._writer is None or self._path is None:
                return RecordingResult(
                    False,
                    "",
                    0,
                    0,
                    0.0,
                    0.0,
                    0,
                    "녹화 중이 아닙니다",
                )

            duration = max(0.0, time.monotonic() - self._started_at)
            path = self._path
            width = self._width
            height = self._height
            fps = self._fps
            frames_written = self._frames_written
            runtime_error = self._runtime_error
            codec_label = self._codec_label

            self._writer.release()
            self._writer = None
            self._previous_frame = None

            valid, validation_message = self._validate_video(
                path,
                width,
                height,
                expected_frames=frames_written,
            )
            success = valid and not runtime_error and frames_written > 0
            message_parts = [codec_label, validation_message]
            if self._fallback_reason:
                message_parts.append(self._fallback_reason)
            if runtime_error:
                message_parts.append(runtime_error)
            message = "; ".join(part for part in message_parts if part)
            return RecordingResult(
                success,
                str(path),
                width,
                height,
                fps,
                duration,
                frames_written,
                message,
            )

    def _write_frame_locked(self, frame, received_at: float):
        target_index = max(
            self._next_timeline_index,
            int(round((received_at - self._started_at) * self._fps)),
        )
        gap = target_index - self._next_timeline_index
        max_gap = max(1, int(round(self._max_fill_gap_seconds * self._fps)))
        if self._previous_frame is not None:
            fill_count = min(gap, max_gap)
            for _ in range(fill_count):
                self._writer.write(self._previous_frame)
                self._frames_written += 1
                self._next_timeline_index += 1
            if gap > max_gap:
                self._started_at = received_at - (self._next_timeline_index / self._fps)

        self._writer.write(frame)
        self._frames_written += 1
        self._next_timeline_index += 1
        self._previous_frame = frame

    def _probe_writer(
        self,
        path: Path,
        codec: str,
        fps: float,
        size: Tuple[int, int],
        frame,
    ) -> bool:
        writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*codec),
            fps,
            size,
        )
        if not writer.isOpened():
            writer.release()
            self._remove_created_file(path)
            return False
        writer.write(frame)
        writer.write(frame)
        writer.release()
        valid, _ = self._validate_video(
            path,
            size[0],
            size[1],
            minimum_bytes=1,
            expected_frames=2,
        )
        self._remove_created_file(path)
        return valid

    def _validate_video(
        self,
        path: Path,
        expected_width: int,
        expected_height: int,
        minimum_bytes: Optional[int] = None,
        expected_frames: int = 1,
    ) -> Tuple[bool, str]:
        threshold = (
            self._minimum_valid_bytes
            if minimum_bytes is None
            else minimum_bytes
        )
        try:
            file_size = path.stat().st_size
        except OSError as exc:
            return False, "파일 확인 실패: %s" % exc
        if file_size < threshold:
            return False, "파일 크기가 너무 작음(%d bytes)" % file_size

        capture = cv2.VideoCapture(str(path))
        try:
            if not capture.isOpened():
                return False, "OpenCV로 결과 파일을 열 수 없음"
            ok, frame = capture.read()
            if not ok or frame is None or frame.size == 0:
                return False, "결과 파일의 첫 프레임을 읽을 수 없음"
            height, width = frame.shape[:2]
            if width != expected_width or height != expected_height:
                return (
                    False,
                    "결과 해상도 불일치: %dx%d" % (width, height),
                )
            reported_frames = int(round(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
            if expected_frames > 1:
                capture.set(cv2.CAP_PROP_POS_FRAMES, expected_frames - 1)
                last_ok, last_frame = capture.read()
                if not last_ok or last_frame is None or last_frame.size == 0:
                    return False, "마지막 기록 프레임을 읽을 수 없음"
            return (
                True,
                "재생 검증 통과(%d bytes, writer %d frames, container %d frames)"
                % (file_size, expected_frames, reported_frames),
            )
        finally:
            capture.release()

    def _unique_base(self, width: int, height: int) -> Path:
        stem = "puppypi_%s_%dx%d" % (
            datetime.now().strftime("%Y%m%d_%H%M%S"),
            width,
            height,
        )
        candidate = self._directory / stem
        suffix = 1
        while any(
            candidate.with_suffix(extension).exists()
            for extension in (".mp4", ".avi", ".mkv")
        ):
            candidate = self._directory / ("%s_%02d" % (stem, suffix))
            suffix += 1
        return candidate

    @staticmethod
    def _remove_created_file(path: Path):
        try:
            if path.is_file():
                path.unlink()
        except OSError:
            pass

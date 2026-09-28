#!/usr/bin/env python3
"""Resolve camera.name_contains to a stable V4L2 path for usb_cam (Quest v2 경로).

stdout에는 장치 경로 한 줄만 출력합니다. 요청한 pixel format과 해상도를
장치가 광고하지 않으면 usb_cam을 띄우기 전에 실패합니다.
"""

import argparse
import sys

import yaml

from camera_stream import CameraStreamer, discover_camera


# usb_cam pixel_format 이름 -> v4l2-ctl --list-formats-ext 의 FourCC
PIXEL_FORMATS = {
    "mjpeg": {"MJPG", "JPEG"},
    "yuyv": {"YUYV"},
    "uyvy": {"UYVY"},
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--pixel-format", required=True, choices=sorted(PIXEL_FORMATS))
    parser.add_argument("--fps", type=float, default=0.0)
    args = parser.parse_args()

    with open(args.config, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    try:
        selection = discover_camera(str(config["camera"]["name_contains"]))
    except (KeyError, TypeError, ValueError, RuntimeError) as exc:
        print("카메라 탐색 실패: %s" % exc, file=sys.stderr)
        return 5

    fourccs = PIXEL_FORMATS[args.pixel_format]
    matches = [
        mode
        for mode in selection.modes
        if mode.pixel_format in fourccs
        and mode.width == args.width
        and mode.height == args.height
    ]
    print(
        "선택 카메라: %s (%s)\n지원 모드:\n%s"
        % (
            selection.description,
            selection.device_path,
            CameraStreamer.format_supported_modes(selection.modes),
        ),
        file=sys.stderr,
    )
    if not matches:
        print(
            "요청 모드 %s %dx%d 를 장치가 광고하지 않습니다. 위 목록의 값으로 "
            "image_width:=, image_height:=, pixel_format:= 를 지정하십시오."
            % (args.pixel_format, args.width, args.height),
            file=sys.stderr,
        )
        return 6
    best_fps = max(mode.fps for mode in matches)
    if args.fps > 0 and best_fps and best_fps < args.fps - 0.01:
        print(
            "경고: 요청 %.1ffps 보다 광고된 최대 %.1ffps가 낮습니다" % (args.fps, best_fps),
            file=sys.stderr,
        )
    print(selection.device_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())

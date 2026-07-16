#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 없이도 사용할 수 있는 지도 검증 산출물 유틸리티."""

import math
import os


def quaternion_to_yaw(x, y, z, w):
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z))


def occupancy_counts(data, free_threshold=25, occupied_threshold=65):
    counts = {'unknown': 0, 'free': 0, 'occupied': 0, 'uncertain': 0}
    for value in data:
        if value < 0:
            counts['unknown'] += 1
        elif value <= free_threshold:
            counts['free'] += 1
        elif value >= occupied_threshold:
            counts['occupied'] += 1
        else:
            counts['uncertain'] += 1
    counts['known'] = counts['free'] + counts['occupied'] + counts['uncertain']
    return counts


def occupancy_to_grayscale(data, width, height,
                           free_threshold=25, occupied_threshold=65):
    """OccupancyGrid 데이터를 위쪽이 +Y인 PGM 픽셀로 변환한다."""
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError('invalid occupancy grid')
    pixels = bytearray()
    for y in range(height - 1, -1, -1):
        offset = y * width
        for x in range(width):
            value = data[offset + x]
            if value < 0:
                pixels.append(205)
            elif value <= free_threshold:
                pixels.append(254)
            elif value >= occupied_threshold:
                pixels.append(0)
            else:
                pixels.append(max(1, min(253, int(round(254 * (100 - value) / 100.0)))))
    return bytes(pixels)


def write_pgm(path, data, width, height):
    pixels = occupancy_to_grayscale(data, width, height)
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    with open(path, 'wb') as stream:
        stream.write(('P5\n%d %d\n255\n' % (width, height)).encode('ascii'))
        stream.write(pixels)


def write_map_yaml(path, image_name, resolution, origin_x, origin_y):
    with open(path, 'w') as stream:
        stream.write(
            'image: %s\nresolution: %.8f\norigin: [%.8f, %.8f, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n' %
            (image_name, resolution, origin_x, origin_y))


def angle_delta(current, previous):
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


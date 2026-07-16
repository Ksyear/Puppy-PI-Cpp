#!/usr/bin/env python3
"""PuppyPi 지도작성 번들을 ROS 설치 없이 분석한다."""

import argparse
import csv
import hashlib
import html
import json
import os
import struct
import sys
import tarfile
import zlib


PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def verify_bundle_checksum(bundle_path):
    checksum_path = bundle_path + '.sha256'
    if not os.path.isfile(checksum_path):
        return False, 'bundle checksum file not present'
    with open(checksum_path) as stream:
        fields = stream.read().strip().split()
    if not fields:
        raise ValueError('empty bundle checksum file')
    actual = sha256_file(bundle_path)
    if actual != fields[0].lower():
        raise ValueError('bundle SHA-256 mismatch')
    return True, actual


def safe_extract(bundle_path, target_parent):
    target_parent = os.path.abspath(target_parent)
    os.makedirs(target_parent, exist_ok=True)
    with tarfile.open(bundle_path, 'r:gz') as archive:
        members = archive.getmembers()
        top_levels = set()
        for member in members:
            normalized = os.path.normpath(member.name)
            if (os.path.isabs(member.name) or normalized.startswith('..') or
                    member.issym() or member.islnk()):
                raise ValueError('unsafe archive member: %s' % member.name)
            destination = os.path.abspath(os.path.join(target_parent, normalized))
            if os.path.commonpath([target_parent, destination]) != target_parent:
                raise ValueError('archive member escapes target: %s' % member.name)
            if normalized and normalized != '.':
                top_levels.add(normalized.split(os.sep)[0])
        if len(top_levels) != 1:
            raise ValueError('bundle must contain exactly one run directory')
        run_dir = os.path.join(target_parent, next(iter(top_levels)))
        if not os.path.isdir(run_dir):
            archive.extractall(target_parent)
    return run_dir


def resolve_run(input_path, extract_root=None):
    input_path = os.path.abspath(os.path.expanduser(input_path))
    checksum = {'present': False, 'detail': ''}
    if os.path.isdir(input_path):
        return input_path, checksum
    if not os.path.isfile(input_path):
        raise ValueError('input does not exist: %s' % input_path)
    if not input_path.endswith(('.tar.gz', '.tgz')):
        raise ValueError('input must be a run directory or .tar.gz bundle')
    checksum['present'], checksum['detail'] = verify_bundle_checksum(input_path)
    parent = extract_root or os.path.dirname(input_path)
    return safe_extract(input_path, parent), checksum


def verify_internal_checksums(run_dir):
    checksum_path = os.path.join(run_dir, 'checksums.sha256')
    if not os.path.isfile(checksum_path):
        return {'present': False, 'verified_files': 0}
    verified = 0
    with open(checksum_path) as stream:
        for line in stream:
            line = line.rstrip('\n')
            if not line:
                continue
            expected, separator, relative = line.partition('  ')
            if not separator:
                raise ValueError('invalid internal checksum line')
            path = os.path.abspath(os.path.join(run_dir, relative))
            if os.path.commonpath([os.path.abspath(run_dir), path]) != os.path.abspath(run_dir):
                raise ValueError('checksum path escapes run directory')
            if not os.path.isfile(path) or sha256_file(path) != expected.lower():
                raise ValueError('internal checksum mismatch: %s' % relative)
            verified += 1
    return {'present': True, 'verified_files': verified}


def read_pgm(path):
    with open(path, 'rb') as stream:
        if stream.readline().strip() != b'P5':
            raise ValueError('only binary P5 PGM is supported')

        def next_data_line():
            while True:
                line = stream.readline()
                if not line:
                    raise ValueError('truncated PGM header')
                line = line.strip()
                if line and not line.startswith(b'#'):
                    return line

        width, height = [int(value) for value in next_data_line().split()]
        maximum = int(next_data_line())
        if maximum != 255:
            raise ValueError('unsupported PGM maximum value')
        pixels = stream.read()
    if len(pixels) != width * height:
        raise ValueError('PGM pixel length mismatch')
    return width, height, pixels


def png_chunk(chunk_type, data):
    checksum = zlib.crc32(chunk_type)
    checksum = zlib.crc32(data, checksum) & 0xffffffff
    return (struct.pack('>I', len(data)) + chunk_type + data +
            struct.pack('>I', checksum))


def write_png(path, width, height, pixels):
    rows = []
    for row in range(height):
        start = row * width
        rows.append(b'\x00' + pixels[start:start + width])
    payload = bytearray(PNG_SIGNATURE)
    payload.extend(png_chunk(b'IHDR', struct.pack(
        '>IIBBBBB', width, height, 8, 0, 0, 0, 0)))
    payload.extend(png_chunk(b'IDAT', zlib.compress(b''.join(rows), 9)))
    payload.extend(png_chunk(b'IEND', b''))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'wb') as stream:
        stream.write(payload)


def load_json(path, default=None):
    if not os.path.isfile(path):
        return default
    with open(path) as stream:
        return json.load(stream)


def load_poses(path):
    if not os.path.isfile(path):
        return []
    with open(path, newline='') as stream:
        return [{
            'elapsed_sec': float(row['elapsed_sec']),
            'x': float(row['x']),
            'y': float(row['y']),
            'yaw': float(row['yaw']),
        } for row in csv.DictReader(stream)]


def write_trajectory_svg(path, map_png_name, width, height, info, poses):
    resolution = float(info.get('resolution', 1.0))
    origin_x = float(info.get('origin_x', 0.0))
    origin_y = float(info.get('origin_y', 0.0))
    if len(poses) > 2000:
        stride = max(1, len(poses) // 2000)
        poses = poses[::stride] + [poses[-1]]
    points = []
    for pose in poses:
        x = (pose['x'] - origin_x) / resolution
        y = height - (pose['y'] - origin_y) / resolution
        points.append((x, y))
    polyline = ' '.join('%.2f,%.2f' % point for point in points)
    markers = ''
    if points:
        markers = (
            '<circle cx="%.2f" cy="%.2f" r="3" fill="#00b050"/>'
            '<circle cx="%.2f" cy="%.2f" r="3" fill="#e53935"/>' %
            (points[0][0], points[0][1], points[-1][0], points[-1][1]))
    document = '''<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d"
 viewBox="0 0 %d %d">
 <image href="%s" x="0" y="0" width="%d" height="%d"/>
 <polyline points="%s" fill="none" stroke="#1976d2" stroke-width="2"
   stroke-linecap="round" stroke-linejoin="round"/>
 %s
</svg>
''' % (width, height, width, height, html.escape(map_png_name), width, height,
       polyline, markers)
    with open(path, 'w') as stream:
        stream.write(document)


def write_timeline(path, snapshot_names):
    encoded = json.dumps(snapshot_names, ensure_ascii=False)
    document = '''<!doctype html>
<meta charset="utf-8">
<title>PuppyPi map timeline</title>
<style>body{font-family:system-ui;margin:2rem;background:#111;color:#eee}
img{image-rendering:pixelated;max-width:min(90vw,800px);border:1px solid #555}
input{width:min(90vw,800px)}</style>
<h1>지도 변화 타임라인</h1><p id="name"></p>
<input id="slider" type="range" min="0" value="0"><br><img id="map">
<script>
const frames=%s, slider=document.querySelector('#slider'), image=document.querySelector('#map');
slider.max=Math.max(0,frames.length-1);
function show(){const value=frames[Number(slider.value)];image.src=value||'';
document.querySelector('#name').textContent=value||'스냅샷 없음';}
slider.addEventListener('input',show);show();
</script>
''' % encoded
    with open(path, 'w') as stream:
        stream.write(document)


def recommendation_for(check):
    name = check.get('name', '')
    recommendations = {
        'scan_messages': 'LiDAR 노드와 scan 토픽 이름을 확인한다.',
        'scan_valid_beams': 'min/max range와 LiDAR 방향·가림을 확인한다.',
        'scan_max_gap': 'CPU 부하와 LiDAR 드라이버 누락 구간을 확인한다.',
        'map_messages': 'lidar_mapping 노드 로그와 ROS Timer를 확인한다.',
        'map_max_gap': '지도 처리 부하와 /use_sim_time 설정을 확인한다.',
        'pose_messages': '유효 빔 수와 스캔 처리 예외를 확인한다.',
        'pose_max_gap': '스캔매칭 처리시간과 CPU 부하를 확인한다.',
        'pose_scan_ratio': '매퍼가 버린 스캔과 유효 빔 수를 확인한다.',
        'scan_frame': 'LiDAR→base 정적 TF와 장착 translation/yaw를 확인한다.',
        'map_free_space': '광선 free-space 적분과 LiDAR 범위 필터를 확인한다.',
        'map_obstacles': '벽 끝점 적분, max range와 laser_yaw를 확인한다.',
        'map_growth': '로봇 이동 여부와 스캔매칭 자세 변화를 확인한다.',
        'known_cells_monotonic': '지도 배열 크기·원점 변경 또는 known 상태 회귀를 확인한다.',
        'stationary_position_drift': '정지 스캔매칭 점수와 LiDAR 진동을 조정한다.',
        'stationary_yaw_drift': 'LiDAR 진동과 각도 스캔매칭 안정성을 확인한다.',
        'pose_step': '급격한 스캔매칭 오정합 구간을 rosbag에서 재생한다.',
        'map_boundary_margin': 'map_size_m을 늘리거나 시작 원점을 조정한다.',
        'return_error': '작은 폐회로에서 누적 드리프트를 확인하고 IMU/오도메트리를 검토한다.',
    }
    return recommendations.get(name, '관련 ROS 로그와 원본 rosbag 구간을 확인한다.')


def generate_report(run_dir, analysis_dir, summary, manifest, checksum_info,
                    internal_checksum, generated):
    verdict = summary.get('verdict', 'FAIL') if summary else 'FAIL'
    checks = summary.get('checks', []) if summary else []
    messages = summary.get('messages', {}) if summary else {}
    map_data = summary.get('map', {}) if summary else {}
    pose = summary.get('pose', {}) if summary else {}
    lines = [
        '# PuppyPi 지도작성 검증 보고서', '',
        '**결론: %s**' % verdict, '',
        '- 실행 ID: `%s`' % (manifest or {}).get('run_id', os.path.basename(run_dir)),
        '- 수집 완료 표식: `%s`' % os.path.isfile(
            os.path.join(run_dir, 'COLLECTION_COMPLETE')),
        '- rosbag 정상 종료: `%s`' % (manifest or {}).get('rosbag_complete', False),
        '- 내부 체크섬 검증 파일: `%s`' % internal_checksum.get('verified_files', 0),
        '- 번들 체크섬: `%s`' % ('검증됨' if checksum_info.get('present') else '파일 없음'),
        '', '## 주요 수치', '',
        '| 항목 | 값 |', '|---|---:|',
        '| Scan 수신율 | %.2f Hz |' % messages.get('scan_rate_hz', 0.0),
        '| Map 수신율 | %.2f Hz |' % messages.get('map_rate_hz', 0.0),
        '| Pose 수신율 | %.2f Hz |' % messages.get('pose_rate_hz', 0.0),
        '| 평균 유효 LiDAR 빔 | %.1f |' % messages.get('average_valid_scan_beams', 0.0),
        '| 최종 지도 커버리지 | %.2f%% |' % map_data.get('final_coverage_percent', 0.0),
        '| 알려진 셀 증가 | %s |' % map_data.get('known_growth_cells', 0),
        '| 추정 이동 경로 | %.3f m |' % pose.get('path_length_m', 0.0),
        '| 시작-종료 변위 | %.3f m |' % pose.get('net_displacement_m', 0.0),
        '| 정지 위치 드리프트 | %.3f m |' % pose.get('stationary_max_drift_m', 0.0),
        '| 최대 위치 순간이동 | %.3f m |' % pose.get('max_position_step_m', 0.0),
        '', '## 자동 검사', '', '| 검사 | 결과 | 내용 |', '|---|---|---|',
    ]
    if checks:
        for check in checks:
            lines.append('| %s | %s | %s |' % (
                check.get('name', ''), check.get('status', ''),
                str(check.get('detail', '')).replace('|', '\\|')))
    else:
        lines.append('| recorder summary | FAIL | summary.json 없음 |')

    problems = [item for item in checks if item.get('status') != 'PASS']
    lines.extend(['', '## 개선 필요사항', ''])
    if problems:
        for check in problems:
            lines.append('- `%s`: %s' % (
                check.get('name', ''), recommendation_for(check)))
    elif summary:
        lines.append('- 자동 검사에서 즉시 수정할 항목이 발견되지 않았습니다.')
        lines.append('- 실제 벽과의 절대 정확도는 시험 환경 영상 또는 실측 치수로 별도 확인해야 합니다.')
    else:
        lines.append('- recorder가 종료되지 않았거나 실행에 실패했습니다. `roslaunch.log`를 확인합니다.')

    lines.extend(['', '## 생성된 자료', ''])
    for label, relative in generated:
        lines.append('- [%s](%s)' % (label, relative))
    lines.append('')
    report_path = os.path.join(analysis_dir, 'report.md')
    with open(report_path, 'w') as stream:
        stream.write('\n'.join(lines))
    return report_path, verdict


def analyze_run(run_dir, bundle_checksum=None):
    run_dir = os.path.abspath(run_dir)
    checksum_info = bundle_checksum or {'present': False, 'detail': ''}
    internal_checksum = verify_internal_checksums(run_dir)
    summary = load_json(os.path.join(run_dir, 'summary.json'))
    manifest = load_json(os.path.join(run_dir, 'manifest.json'), {})
    poses = load_poses(os.path.join(run_dir, 'pose.csv'))
    analysis_dir = os.path.join(run_dir, 'analysis')
    os.makedirs(analysis_dir, exist_ok=True)
    generated = []

    final_map = os.path.join(run_dir, 'final_map.pgm')
    if os.path.isfile(final_map):
        width, height, pixels = read_pgm(final_map)
        final_png = os.path.join(analysis_dir, 'final_map.png')
        write_png(final_png, width, height, pixels)
        generated.append(('최종 지도 PNG', 'final_map.png'))
        info = ((summary or {}).get('map', {}).get('info') or {
            'width': width, 'height': height, 'resolution': 1.0,
            'origin_x': 0.0, 'origin_y': 0.0})
        trajectory = os.path.join(analysis_dir, 'trajectory.svg')
        write_trajectory_svg(
            trajectory, 'final_map.png', width, height, info, poses)
        generated.append(('지도와 추정 이동 경로', 'trajectory.svg'))

    snapshots_dir = os.path.join(run_dir, 'maps')
    snapshot_output = os.path.join(analysis_dir, 'snapshots')
    snapshot_names = []
    if os.path.isdir(snapshots_dir):
        for name in sorted(os.listdir(snapshots_dir)):
            if not name.endswith('.pgm'):
                continue
            width, height, pixels = read_pgm(os.path.join(snapshots_dir, name))
            png_name = os.path.splitext(name)[0] + '.png'
            write_png(os.path.join(snapshot_output, png_name), width, height, pixels)
            snapshot_names.append('snapshots/' + png_name)
    if snapshot_names:
        timeline = os.path.join(analysis_dir, 'timeline.html')
        write_timeline(timeline, snapshot_names)
        generated.append(('지도 변화 타임라인', 'timeline.html'))

    report_path, verdict = generate_report(
        run_dir, analysis_dir, summary, manifest, checksum_info,
        internal_checksum, generated)
    result = {
        'run_dir': run_dir,
        'verdict': verdict,
        'report': report_path,
        'bundle_checksum': checksum_info,
        'internal_checksum': internal_checksum,
    }
    with open(os.path.join(analysis_dir, 'result.json'), 'w') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write('\n')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='PuppyPi ROS1 지도작성 실행 번들을 오프라인 분석합니다.')
    parser.add_argument('input', help='실행 디렉터리 또는 .tar.gz 번들')
    parser.add_argument('--extract-root', help='번들을 풀 디렉터리')
    args = parser.parse_args(argv)
    try:
        run_dir, checksum = resolve_run(args.input, args.extract_root)
        result = analyze_run(run_dir, checksum)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print('[분석 실패] %s' % exc, file=sys.stderr)
        return 2
    print('RESULT=%s' % result['verdict'])
    print('REPORT=%s' % result['report'])
    return 0


if __name__ == '__main__':
    sys.exit(main())


#!/usr/bin/env python3

import csv
import json
import os
import sys
import tempfile
import unittest


TOOLS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, TOOLS_DIR)

import analyze_mapping_run as analyzer


class AnalyzeMappingRunTest(unittest.TestCase):

    def make_run(self, root):
        run = os.path.join(root, '20260716_120000_test')
        os.makedirs(os.path.join(run, 'maps'))
        pixels = bytes([205, 254, 0, 254] * 4)
        pgm = b'P5\n4 4\n255\n' + pixels
        for path in ('final_map.pgm', 'maps/map_0001.pgm'):
            with open(os.path.join(run, path), 'wb') as stream:
                stream.write(pgm)
        with open(os.path.join(run, 'pose.csv'), 'w', newline='') as stream:
            writer = csv.DictWriter(
                stream, fieldnames=['elapsed_sec', 'ros_time', 'x', 'y', 'yaw'])
            writer.writeheader()
            writer.writerow({'elapsed_sec': 0, 'ros_time': 1, 'x': 0, 'y': 0, 'yaw': 0})
            writer.writerow({'elapsed_sec': 1, 'ros_time': 2, 'x': 1, 'y': 1, 'yaw': 0})
        summary = {
            'verdict': 'PASS',
            'messages': {
                'scan_rate_hz': 10, 'map_rate_hz': 1, 'pose_rate_hz': 10,
                'average_valid_scan_beams': 300,
            },
            'map': {
                'known_growth_cells': 8,
                'final_coverage_percent': 50,
                'info': {
                    'width': 4, 'height': 4, 'resolution': 1,
                    'origin_x': 0, 'origin_y': 0,
                },
            },
            'pose': {
                'path_length_m': 1.4, 'net_displacement_m': 1.4,
                'stationary_max_drift_m': 0.01, 'max_position_step_m': 0.1,
            },
            'checks': [{'name': 'map_messages', 'status': 'PASS', 'detail': '2 messages'}],
        }
        with open(os.path.join(run, 'summary.json'), 'w') as stream:
            json.dump(summary, stream)
        with open(os.path.join(run, 'manifest.json'), 'w') as stream:
            json.dump({'run_id': os.path.basename(run), 'rosbag_complete': True}, stream)
        with open(os.path.join(run, 'COLLECTION_COMPLETE'), 'w') as stream:
            stream.write('ok\n')
        return run

    def test_generates_portable_report_and_visuals(self):
        with tempfile.TemporaryDirectory() as directory:
            run = self.make_run(directory)
            result = analyzer.analyze_run(run)
            self.assertEqual(result['verdict'], 'PASS')
            analysis = os.path.join(run, 'analysis')
            self.assertTrue(os.path.isfile(os.path.join(analysis, 'report.md')))
            self.assertTrue(os.path.isfile(os.path.join(analysis, 'trajectory.svg')))
            self.assertTrue(os.path.isfile(os.path.join(analysis, 'timeline.html')))
            with open(os.path.join(analysis, 'final_map.png'), 'rb') as stream:
                self.assertEqual(stream.read(8), analyzer.PNG_SIGNATURE)

    def test_rejects_bad_checksum(self):
        with tempfile.TemporaryDirectory() as directory:
            bundle = os.path.join(directory, 'run.tar.gz')
            with open(bundle, 'wb') as stream:
                stream.write(b'not a bundle')
            with open(bundle + '.sha256', 'w') as stream:
                stream.write('0' * 64 + '  run.tar.gz\n')
            with self.assertRaises(ValueError):
                analyzer.verify_bundle_checksum(bundle)


if __name__ == '__main__':
    unittest.main()

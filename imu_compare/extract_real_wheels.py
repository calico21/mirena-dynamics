#!/usr/bin/env python3
"""Extract /sensors/wheel_speeds from a rosbag2 mcap into CSV.

The real bag's /sensors/imu/odom only starts 275 s in, but the wheel-speed
topic spans the whole drive, so it is the better real-speed reference.  Units
are checked against odometry in speed_analysis.py.
"""
from __future__ import annotations

import argparse
import csv
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message

from mirena_common.msg import WheelSpeeds


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('bag')
    p.add_argument('--out', default='real_wheels.csv')
    p.add_argument('--topic', default='/sensors/wheel_speeds')
    a = p.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=a.bag, storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[a.topic]))

    with open(a.out, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'hx', 'rl', 'rr'])
        n = 0
        while reader.has_next():
            _topic, data, t = reader.read_next()
            m = deserialize_message(data, WheelSpeeds)
            h = m.header.stamp
            w.writerow([f'{t * 1e-9:.9f}', f'{h.sec + h.nanosec * 1e-9:.9f}',
                        m.rl, m.rr])
            n += 1
    print(f'wrote {n} rows -> {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

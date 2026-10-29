#!/usr/bin/env python3
"""Extract /sensors/imu/odom from a rosbag2 mcap into CSV.

Used to put the real car's measured speed next to the simulator's
/debug/state/car speed, so the aero patch can be judged on speed as well as on
the IMU channels.  Reading the bag directly is much faster than playing it.
"""
from __future__ import annotations

import argparse
import csv
import sys

import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('bag')
    p.add_argument('--out', default='real_odom.csv')
    p.add_argument('--topic', default='/sensors/imu/odom')
    a = p.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=a.bag, storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[a.topic]))

    with open(a.out, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'hx', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw',
                    'vx', 'vy', 'vz', 'wx', 'wy', 'wz'])
        n = 0
        while reader.has_next():
            _topic, data, t = reader.read_next()
            m = deserialize_message(data, Odometry)
            h = m.header.stamp
            w.writerow([
                f'{t * 1e-9:.9f}', f'{h.sec + h.nanosec * 1e-9:.9f}',
                m.pose.pose.position.x, m.pose.pose.position.y,
                m.pose.pose.position.z,
                m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                m.pose.pose.orientation.z, m.pose.pose.orientation.w,
                m.twist.twist.linear.x, m.twist.twist.linear.y,
                m.twist.twist.linear.z,
                m.twist.twist.angular.x, m.twist.twist.angular.y,
                m.twist.twist.angular.z,
            ])
            n += 1
    print(f'wrote {n} rows -> {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

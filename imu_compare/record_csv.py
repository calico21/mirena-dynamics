#!/usr/bin/env python3
"""Record /sensors/imu (best effort) and a control topic to CSV.

Writes two files: <prefix>_imu.csv and <prefix>_control.csv.  Both use a
single t0 per process so t_rel is directly comparable between the two files.

Usage:
  python3 record_csv.py --out-prefix real \
      --imu-topic /sensors/imu --control-topic /control/can \
      --control-steer-units deg --duration 415
"""
from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Imu

from mirena_common.msg import CarControl


class Recorder(Node):
    def __init__(self, args):
        super().__init__('imu_csv_recorder')
        self.args = args
        # t_rel is a wall-clock timeline shared by both topics.  Header stamps
        # are kept in t_unix for provenance but the two topics can live in
        # different clock domains (the sim IMU uses /clock while the adapter
        # forwards the bag's epoch stamps).  A common wall clock keeps the CSV
        # alignable without post-processing.
        self.t0 = None
        self.n_imu = 0
        self.n_ctl = 0
        self.last_imu_wall = None
        self.last_ctl_wall = None

        self.f_imu = open(f'{args.out_prefix}_imu.csv', 'w', newline='')
        self.f_ctl = open(f'{args.out_prefix}_control.csv', 'w', newline='')
        self.w_imu = csv.writer(self.f_imu)
        self.w_ctl = csv.writer(self.f_ctl)
        self.w_imu.writerow(['t_unix', 't_rel', 'ax', 'ay', 'az',
                             'gx', 'gy', 'gz',
                             'qx', 'qy', 'qz', 'qw', 'frame_id'])
        self.w_ctl.writerow(['t_unix', 't_rel', 'gas', 'steer_raw',
                             'steer_rad', 'frame_id'])

        imu_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        ctl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Imu, args.imu_topic, self.on_imu, imu_qos)
        self.create_subscription(CarControl, args.control_topic,
                                 self.on_control, ctl_qos)

        self.started = time.monotonic()
        self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f'recording {args.imu_topic} (best effort) and '
            f'{args.control_topic} (reliable) -> {args.out_prefix}_*.csv')

    def stamp(self, msg):
        s = msg.header.stamp
        return float(s.sec) + float(s.nanosec) * 1e-9

    def rel(self, t_wall):
        if self.t0 is None:
            self.t0 = t_wall
        return t_wall - self.t0

    def on_imu(self, msg):
        t = self.stamp(msg)
        t_wall = time.monotonic()
        a = msg.linear_acceleration
        g = msg.angular_velocity
        q = msg.orientation
        self.w_imu.writerow([
            f'{t:.9f}', f'{self.rel(t_wall):.9f}',
            f'{a.x:.9g}', f'{a.y:.9g}', f'{a.z:.9g}',
            f'{g.x:.9g}', f'{g.y:.9g}', f'{g.z:.9g}',
            f'{q.x:.9g}', f'{q.y:.9g}', f'{q.z:.9g}', f'{q.w:.9g}',
            msg.header.frame_id,
        ])
        self.n_imu += 1
        self.last_imu_wall = time.monotonic()
        if self.n_imu % 200 == 0:
            self.f_imu.flush()

    def on_control(self, msg):
        t = self.stamp(msg)
        t_wall = time.monotonic()
        raw = float(msg.steer_angle)
        rad = math.radians(raw) if self.args.control_steer_units == 'deg' else raw
        rad *= self.args.steer_sign
        self.w_ctl.writerow([
            f'{t:.9f}', f'{self.rel(t_wall):.9f}',
            f'{float(msg.gas):.9g}', f'{raw:.9g}', f'{rad:.9g}',
            msg.header.frame_id,
        ])
        self.n_ctl += 1
        self.last_ctl_wall = time.monotonic()
        if self.n_ctl % 200 == 0:
            self.f_ctl.flush()

    def tick(self):
        elapsed = time.monotonic() - self.started
        if self.args.duration > 0 and elapsed >= self.args.duration:
            self.get_logger().info(
                f'duration reached: {self.n_imu} IMU, {self.n_ctl} control')
            raise SystemExit(0)
        if int(elapsed) % 10 == 0:
            self.get_logger().info(
                f't={elapsed:.0f}s imu={self.n_imu} control={self.n_ctl}')

    def close(self):
        self.f_imu.flush()
        self.f_ctl.flush()
        self.f_imu.close()
        self.f_ctl.close()
        print(f'wrote {self.n_imu} IMU rows -> {self.args.out_prefix}_imu.csv')
        print(f'wrote {self.n_ctl} control rows -> '
              f'{self.args.out_prefix}_control.csv')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out-prefix', required=True)
    p.add_argument('--imu-topic', default='/sensors/imu')
    p.add_argument('--control-topic', default='/control')
    p.add_argument('--control-steer-units', choices=['deg', 'rad'], default='rad')
    p.add_argument('--steer-sign', type=float, default=1.0)
    p.add_argument('--duration', type=float, default=0.0,
                   help='stop after N seconds (0 = run until Ctrl-C)')
    args = p.parse_args()

    rclpy.init()
    rec = Recorder(args)

    def stop(_sig, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(rec)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rec.close()
        rec.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

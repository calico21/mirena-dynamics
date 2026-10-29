#!/usr/bin/env python3
"""Log the simulator's ground-truth car state to CSV for offline analysis.

Subscribes to /debug/state/car (400 Hz) and keeps every 4th sample, giving
~100 Hz: sim time, pose, and body velocities (u, v, omega).  This is only used
to show whether the aero patch actually caps the sim's speed; the IMU
comparison itself stays on the recorded /sensors/imu stream.
"""
from __future__ import annotations

import argparse
import csv
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from mirena_common.msg import Car


class StateRecorder(Node):
    def __init__(self, path: str, duration: float) -> None:
        super().__init__('state_recorder')
        self.path = path
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'psi', 'u', 'v', 'omega'])
        self.n = 0
        self.duration = duration
        self.start = time.monotonic()
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Car, '/debug/state/car', self.cb, qos)
        self.create_timer(0.5, self.tick)
        self.get_logger().info(f'logging /debug/state/car -> {path}')

    def cb(self, msg: Car) -> None:
        self.n += 1
        if self.n % 4 != 1:
            return
        st = msg.header.stamp
        t = st.sec + st.nanosec * 1e-9
        self.w.writerow([f'{t:.6f}', msg.x, msg.y, msg.psi,
                         msg.u, msg.v, msg.omega])
        if self.n % 400 == 0:
            self.f.flush()

    def tick(self) -> None:
        if self.duration > 0 and time.monotonic() - self.start >= self.duration:
            self.get_logger().info(f'duration reached: {self.n} messages')
            raise SystemExit(0)

    def close(self) -> None:
        self.f.flush()
        self.f.close()


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('--out', default='sim_state.csv')
    p.add_argument('--duration', type=float, default=0.0)
    a = p.parse_args()
    rclpy.init()
    node = StateRecorder(a.out, a.duration)

    def stop(_sig, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

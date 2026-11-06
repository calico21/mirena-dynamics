#!/usr/bin/env python3
"""Convert /control/can (raw bus degrees) into the sim's /control (radians).

The bag's /control/can steer_angle is the raw CAN signal in Grad, despite the
CarControl.msg comment saying radians.  The simulator's VehicleBody3D.steering
is in radians with MAX_STEER = 30 deg.  Remapping without this conversion
feeds +/-30 rad into the sim.
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy)

from mirena_common.msg import CarControl


class Adapter(Node):
    def __init__(self, args):
        super().__init__('control_adapter')
        self.args = args
        self.n = 0
        self.warned = False
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(CarControl, args.out, qos)
        self.create_subscription(CarControl, args.inp, self.cb, qos)
        self.started = time.monotonic()
        self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f'{args.inp} -> {args.out}  steer_units={args.steer_units} '
            f'sign={args.steer_sign:+.0f} scale={args.steer_scale}')

    def cb(self, msg):
        out = CarControl()
        out.header = msg.header
        out.gas = float(msg.gas)
        steer = float(msg.steer_angle)
        if self.args.steer_units == 'deg':
            steer = math.radians(steer)
        out.steer_angle = self.args.steer_sign * steer * self.args.steer_scale
        if abs(out.steer_angle) > 0.7 and not self.warned:
            self.get_logger().warn(
                f'steer {out.steer_angle:+.3f} rad exceeds 0.7 rad; '
                'check --steer-units')
            self.warned = True
        self.pub.publish(out)
        self.n += 1
        if self.n % 5000 == 0:
            self.get_logger().info(f'forwarded {self.n} messages')

    def tick(self):
        elapsed = time.monotonic() - self.started
        if self.args.duration > 0 and elapsed >= self.args.duration:
            self.get_logger().info(f'duration reached: {self.n} messages')
            raise SystemExit(0)

    def close(self):
        print(f'adapter forwarded {self.n} messages')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--in', dest='inp', default='/control_raw')
    p.add_argument('--out', default='/control')
    p.add_argument('--steer-units', choices=['deg', 'rad'], default='deg')
    p.add_argument('--steer-sign', type=float, default=1.0)
    p.add_argument('--steer-scale', type=float, default=1.0)
    p.add_argument('--duration', type=float, default=0.0)
    args = p.parse_args()

    rclpy.init()
    node = Adapter(args)

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

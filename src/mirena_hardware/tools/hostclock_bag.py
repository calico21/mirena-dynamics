#!/usr/bin/env python3
"""Copy the state node's input topics into one clock domain (host time).

The IMU driver stamps headers from GPS hardware time while the rest of the car
uses the host clock. On this bag the host clock was slewing for the first
~150 s, so the offset goes from -64 ms to -1 ms (see undo_clock_skew.py for the
same fault on other bags). Replaying that as-is makes the encoder/INS
time-sync predict run ahead of the IMU headers, so most IMU predictions are
rejected.

This shifts only the /sensors/imu headers by the measured, smoothed
arrival-header offset, preserving the 10 ms sample cadence, and copies the
other input topics untouched. Output keeps the original arrival times.

Usage: hostclock_bag.py IN_BAG OUT_BAG
"""
import sys
from importlib import import_module

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message

IMU = "/sensors/imu"
COPY = ["/sensors/imu", "/sensors/imu/odom", "/sensors/wheel_speeds", "/control/can"]
BIN_S = 1.0


def open_reader(uri):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    return r


def main():
    src, dst = sys.argv[1], sys.argv[2]

    reader = open_reader(src)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_cls = {}
    for name, typ in types.items():
        pkg, _, kind = typ.split("/")
        msg_cls[name] = getattr(import_module(f"{pkg}.msg"), kind)

    # ---- pass 1: measure the smoothed IMU header-to-host offset -------------
    stamps, skews = [], []
    while reader.has_next():
        topic, data, arrival = reader.read_next()
        if topic != IMU:
            continue
        msg = deserialize_message(data, msg_cls[IMU])
        h = msg.header.stamp
        stamps.append(h.sec + h.nanosec * 1e-9)
        skews.append(arrival / 1e9 - (h.sec + h.nanosec * 1e-9))
    stamps = np.asarray(stamps)
    skews = np.asarray(skews)
    if stamps.size == 0:
        sys.exit(f"no {IMU} messages found")

    t0 = stamps[0]
    nb = int((stamps[-1] - t0) // BIN_S) + 1
    centers, medians = [], []
    for b in range(nb):
        m = (stamps >= t0 + b * BIN_S) & (stamps < t0 + (b + 1) * BIN_S)
        if m.any():
            centers.append(t0 + (b + 0.5) * BIN_S)
            medians.append(float(np.median(skews[m])))
    centers = np.asarray(centers)
    medians = np.asarray(medians)
    shift = np.interp(stamps, centers, medians)

    print(f"{IMU}: {stamps.size} samples, skew {medians[0]*1000:+.1f} ms -> "
          f"{medians[-1]*1000:+.1f} ms over {stamps[-1]-stamps[0]:.1f} s")
    residual = skews - shift
    print(f"residual after shift: median {np.median(residual)*1000:+.2f} ms, "
          f"p95 {np.percentile(np.abs(residual), 95)*1000:.2f} ms")

    # ---- pass 2: write the input topics ------------------------------------
    reader = open_reader(src)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=dst, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    for t in reader.get_all_topics_and_types():
        if t.name not in COPY:
            continue
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=t.name, type=t.type,
            serialization_format=t.serialization_format,
            offered_qos_profiles=t.offered_qos_profiles))

    n = 0
    while reader.has_next():
        topic, data, arrival = reader.read_next()
        if topic not in COPY:
            continue
        if topic == IMU:
            msg = deserialize_message(data, msg_cls[IMU])
            h = msg.header.stamp
            t = h.sec + h.nanosec * 1e-9
            add_ns = int(np.interp(t, centers, medians) * 1e9)
            total = h.sec * 1_000_000_000 + h.nanosec + add_ns
            msg.header.stamp.sec = total // 1_000_000_000
            msg.header.stamp.nanosec = total % 1_000_000_000
            data = serialize_message(msg)
        writer.write(topic, data, arrival)
        n += 1
    print(f"wrote {n} messages to {dst}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Put a pre-fix bag into a single clock domain, so it replays like the fixed car.

What is wrong with these bags
-----------------------------
Two independent faults, which happen to cancel on the "header minus arrival"
metric and so look like one:

  * The IMU driver stamped headers from GPS hardware time while everything else
    on the car ran on the host clock, and the host clock was ahead of GPS. On
    rosbag2_2026_08_05-20_26_23 that is 2.026 s. So a pose and a cone frame
    describing the same instant carry headers 2.026 s apart, and mirena_slam's
    ApproximateTime pairs each cone frame with a pose from 2 s later -- about
    16 m at 7.8 m/s, with delta_ms reading ~0 so no guard fires.

  * mirena_lidar held every MSOP packet until an IMU header stamp caught up with
    a host-clock arrival stamp, which with that same 2.026 s skew meant holding
    each packet 2.026 s. Cloud headers are therefore correct host-clock times
    but the messages were *published* 2.133 s late (2.026 s gate + 0.107 s to
    assemble one rotation).

retime_bag.py cannot repair this. It shifts every arrival time by one constant,
which fixes /clock against the headers but does nothing about headers
disagreeing with each other -- the skew is inside the bag, not between the bag
and the clock. Its median is dominated by the 300 Hz of IMU-family messages, so
it aligns /clock to the IMU family and leaves the cloud 2 s adrift.

What this does instead
----------------------
  1. Adds the measured skew to the *headers* of the GPS-stamped topics, moving
     them into the host clock domain where the cloud headers already live.
     After this every header in the bag means the same thing.

  2. Subtracts the same skew from the *arrival* times of /sensors/lidar/points,
     undoing the gate delay. Without this the clouds still arrive 2.13 s after
     the poses they belong to, and mirena_slam's SyncPolicy(100) only holds 1.0
     s of Car messages at 100 Hz -- every pair would be evicted before its cloud
     landed. The per-cloud assembly time is preserved, so clouds still arrive
     one rotation after their stamp, exactly as the fixed driver publishes them.

Point clouds are copied as raw bytes and never deserialised, so a 2.6 GB bag
costs no memory. Because step 2 moves one topic earlier, the output is produced
by merging two passes over the input rather than sorting anything in memory.

Usage:
    python3 undo_clock_skew.py IN_BAG OUT_BAG [--skew SECONDS]
"""
import argparse
import statistics
import sys

from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import (ConverterOptions, SequentialReader, SequentialWriter,
                        StorageOptions, TopicMetadata)
from rosidl_runtime_py.utilities import get_message

# Topics the VectorNav driver stamped from GPS time. Everything else on the car
# was already on the host clock.
GPS_STAMPED = {
    '/sensors/imu',
    '/sensors/lidar/imu',
    '/sensors/imu/odom',
    '/sensors/gps/fix',
}
LIDAR_TOPIC = '/sensors/lidar/points'


def open_reader(uri):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id='mcap'), ConverterOptions('', ''))
    return reader


def measure_skew(uri, sample=40000):
    """How far the host clock ran ahead of GPS, in seconds.

    Measured only on the GPS-stamped topics, as the median of arrival minus
    header. Those are published immediately, so the difference is the clock
    skew plus a sub-millisecond transport latency.
    """
    reader = open_reader(uri)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    skews = []
    seen = 0
    while reader.has_next() and seen < sample:
        topic, data, arrival = reader.read_next()
        if topic not in GPS_STAMPED:
            continue
        seen += 1
        msg = deserialize_message(data, get_message(types[topic]))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            continue
        skews.append(arrival / 1e9 - stamp)
    if not skews:
        sys.exit(f'no messages found on {sorted(GPS_STAMPED)}; is this a pre-fix bag?')
    return statistics.median(skews), len(skews)


def shift_header(data, msg_type, skew_ns):
    msg = deserialize_message(data, msg_type)
    total = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec + skew_ns
    msg.header.stamp.sec = total // 1_000_000_000
    msg.header.stamp.nanosec = total % 1_000_000_000
    return serialize_message(msg)


def lidar_stream(uri, skew_ns):
    """Point clouds only, delivered earlier by the gate delay. Bytes untouched."""
    reader = open_reader(uri)
    while reader.has_next():
        topic, data, arrival = reader.read_next()
        if topic == LIDAR_TOPIC:
            yield arrival - skew_ns, topic, data


def other_stream(uri, skew_ns, types):
    """Everything else, with GPS-stamped headers moved into the host domain."""
    reader = open_reader(uri)
    while reader.has_next():
        topic, data, arrival = reader.read_next()
        if topic == LIDAR_TOPIC:
            continue
        if topic in GPS_STAMPED:
            data = shift_header(data, get_message(types[topic]), skew_ns)
        yield arrival, topic, data


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input')
    parser.add_argument('output')
    parser.add_argument('--skew', type=float, default=None,
                        help='host-ahead-of-GPS seconds; measured from the bag if omitted')
    args = parser.parse_args()

    if args.skew is None:
        skew, n = measure_skew(args.input)
        print(f'measured host-ahead-of-GPS skew over {n} messages: {skew:+.3f} s')
    else:
        skew = args.skew
        print(f'using supplied skew: {skew:+.3f} s')
    skew_ns = int(skew * 1e9)

    reader = open_reader(args.input)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    writer = SequentialWriter()
    writer.open(StorageOptions(uri=args.output, storage_id='mcap'), ConverterOptions('', ''))
    for topic in reader.get_all_topics_and_types():
        writer.create_topic(TopicMetadata(
            id=0, name=topic.name, type=topic.type,
            serialization_format=topic.serialization_format,
            offered_qos_profiles=topic.offered_qos_profiles))

    # Two sorted streams, merged. Only the lidar topic moves, so this is a
    # two-way merge rather than a sort, and nothing but one message per stream
    # is ever resident.
    a = lidar_stream(args.input, skew_ns)
    b = other_stream(args.input, skew_ns, types)
    ha = next(a, None)
    hb = next(b, None)
    count = 0
    while ha is not None or hb is not None:
        if hb is None or (ha is not None and ha[0] <= hb[0]):
            t, topic, data = ha
            ha = next(a, None)
        else:
            t, topic, data = hb
            hb = next(b, None)
        writer.write(topic, data, t)
        count += 1
        if count % 20000 == 0:
            print(f'  {count} messages...', flush=True)

    print(f'wrote {count} messages to {args.output}')
    print('\nHeaders are now all host-clock. Replay with:')
    print(f'    ros2 bag play {args.output} --clock')
    print('and run the stack with use_sim_time:=true.')


if __name__ == '__main__':
    main()

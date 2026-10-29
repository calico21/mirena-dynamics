#!/usr/bin/env python3
"""Put a bag's play clock onto the same time base as its message headers.

Why this is needed
------------------
The VectorNav driver stamps message headers from GPS hardware time, which is
true UTC to sub-microsecond. rosbag2 stamps each message's *arrival* from the
host clock. If the Jetson's clock is not disciplined to GPS, those two disagree
by however far the host has drifted -- 2.025 s on rosbag2_2026_08_05-20_26_23,
1.399 s on rosbag2_2026_08_05-19_16_17.

That gap is invisible live, because almost everything on the car works off
message headers. It becomes visible the moment you replay, because
`ros2 bag play --clock` derives /clock from the *arrival* times. Nodes that
stamp from headers (mirena_state's odom->base) then publish transforms a couple
of seconds behind nodes that stamp from now() (mirena_slam's map->odom), and
every lookup spanning both fails with

    Lookup would require extrapolation into the future

Shifting the arrival times by the measured offset puts /clock back in the
headers' time base. One constant for the whole bag, so the message order is
exactly as recorded.

The real fix is to discipline the Jetson clock to GPS (gpsd + chrony) so the
two agree at the source; this is for bags already on disk.

Usage:
    python3 retime_bag.py IN_BAG OUT_BAG [--offset SECONDS]
"""
import argparse
import statistics
import sys

from rosbag2_py import (SequentialReader, SequentialWriter, StorageOptions,
                        ConverterOptions, TopicMetadata)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def open_reader(uri):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id='mcap'), ConverterOptions('', ''))
    return reader


def measure_offset(uri, sample=20000):
    """Median of (header.stamp - arrival) over the first `sample` messages.

    Static transforms are skipped: /tf_static is latched, so the recorder picks
    it up whenever it happens to subscribe, and its arrival time says nothing
    about clock skew.
    """
    reader = open_reader(uri)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    skews = []
    seen = 0
    while reader.has_next() and seen < sample:
        topic, data, arrival = reader.read_next()
        seen += 1
        if topic == '/tf_static':
            continue
        try:
            msg = deserialize_message(data, get_message(types[topic]))
        except Exception:
            continue
        header = getattr(msg, 'header', None)
        if header is None:
            continue
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            continue
        skews.append(stamp - arrival / 1e9)
    if not skews:
        sys.exit("no stamped messages found; nothing to measure")
    return statistics.median(skews), len(skews)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input')
    parser.add_argument('output')
    parser.add_argument('--offset', type=float, default=None,
                        help='seconds to add to every arrival time; measured from the bag if omitted')
    args = parser.parse_args()

    if args.offset is None:
        offset, n = measure_offset(args.input)
        print(f"measured header-vs-arrival offset over {n} messages: {offset:+.3f} s")
    else:
        offset = args.offset
        print(f"using supplied offset: {offset:+.3f} s")

    if abs(offset) < 1e-3:
        print("that is under a millisecond -- this bag does not need retiming")

    offset_ns = int(offset * 1e9)
    reader = open_reader(args.input)
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=args.output, storage_id='mcap'), ConverterOptions('', ''))
    for topic in reader.get_all_topics_and_types():
        writer.create_topic(TopicMetadata(
            id=0, name=topic.name, type=topic.type,
            serialization_format=topic.serialization_format,
            offered_qos_profiles=topic.offered_qos_profiles))

    count = 0
    while reader.has_next():
        topic, data, arrival = reader.read_next()
        writer.write(topic, data, arrival + offset_ns)
        count += 1
    print(f"wrote {count} messages to {args.output}")


if __name__ == '__main__':
    main()

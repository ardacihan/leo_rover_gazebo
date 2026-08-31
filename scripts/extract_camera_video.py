#!/usr/bin/env python3
"""Extract each robot's full camera stream from a run's rosbag into a
browser-playable H.264 MP4.

Runs inside the sim container (needs rosbag2_py + cv2):
    python3 scripts/extract_camera_video.py <run_dir> [--robots leo1 leo2]

Writes <run_dir>/camera_<robot>.mp4 at the stream's native rate, with a
sim-time stamp burned into the corner so the video can be lined up with the
map timelapse.
"""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml


def open_reader(bag_dir):
    import rosbag2_py

    with open(os.path.join(bag_dir, "metadata.yaml")) as f:
        meta = yaml.safe_load(f)["rosbag2_bagfile_information"]
    compression = meta.get("compression_format") or ""
    if compression:
        reader = rosbag2_py.SequentialCompressionReader()
    else:
        reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage, converter)
    return reader


def image_to_bgr(msg):
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("rgb8", "bgr8"):
        img = data.reshape(msg.height, msg.step // 3, 3)[:, : msg.width]
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    if msg.encoding == "mono8":
        img = data.reshape(msg.height, msg.step)[:, : msg.width]
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unhandled encoding {msg.encoding}")


def extract(run_dir, robots):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    bag_dir = os.path.join(run_dir, "bag")
    wanted = {f"/{r}/camera/image": r for r in robots}

    # First pass: count frames and time span per topic so the encoded fps
    # matches the recorded rate and the video spans the whole run.
    reader = open_reader(bag_dir)
    counts = {}
    spans = {}
    while reader.has_next():
        topic, _, t_ns = reader.read_next()
        if topic in wanted:
            counts[topic] = counts.get(topic, 0) + 1
            lo, hi = spans.get(topic, (t_ns, t_ns))
            spans[topic] = (min(lo, t_ns), max(hi, t_ns))
    del reader

    writers = {}
    t0 = {}
    for topic, robot in wanted.items():
        n = counts.get(topic, 0)
        if n < 2:
            print(f"WARN: no frames for {topic}", file=sys.stderr)
            continue
        span_s = max(1e-3, (spans[topic][1] - spans[topic][0]) / 1e9)
        fps = max(1.0, n / span_s)
        out = os.path.join(run_dir, f"camera_{robot}.mp4")
        writers[topic] = {"path": out, "fps": fps, "writer": None, "n": 0}
        t0[topic] = spans[topic][0]

    reader = open_reader(bag_dir)
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        slot = writers.get(topic)
        if slot is None:
            continue
        msg = deserialize_message(raw, Image)
        img = image_to_bgr(msg)
        if slot["writer"] is None:
            h, w = img.shape[:2]
            slot["writer"] = cv2.VideoWriter(
                slot["path"], cv2.VideoWriter_fourcc(*"avc1"), slot["fps"], (w, h)
            )
            if not slot["writer"].isOpened():
                raise RuntimeError(f"could not open H.264 writer for {slot['path']}")
        rel = (t_ns - t0[topic]) / 1e9
        cv2.putText(
            img, f"t={rel:6.1f}s", (8, img.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA,
        )
        cv2.putText(
            img, f"t={rel:6.1f}s", (8, img.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA,
        )
        slot["writer"].write(img)
        slot["n"] += 1

    for topic, slot in writers.items():
        if slot["writer"] is not None:
            slot["writer"].release()
            print(f"{slot['path']}: {slot['n']} frames @ {slot['fps']:.1f} fps")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--robots", nargs="+", default=["leo1", "leo2"])
    args = ap.parse_args()
    extract(args.run_dir, args.robots)


if __name__ == "__main__":
    main()

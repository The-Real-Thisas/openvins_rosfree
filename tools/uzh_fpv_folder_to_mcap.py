#!/usr/bin/env python3
"""Convert a UZH-FPV sequence folder into a single MCAP file.

UZH-FPV ships each sequence as plain text + a PNG folder rather than a EuRoC
`mav0/` tree, so it needs its own reader. The output channels match what
`euroc_folder_to_mcap.py` produces (ros1msg encoding) so the C++ runner keeps a
single ingestion path:
  /snappy_imu             sensor_msgs/Imu
  /snappy_cam/stereo_l    sensor_msgs/CompressedImage  (PNG bytes embedded)
  /snappy_cam/stereo_r    sensor_msgs/CompressedImage
  /gt/pose                geometry_msgs/PoseStamped    (from groundtruth.txt)

The camera/imu topics are the `rostopic` values in the snapdragon kalibr chains
(config/uzhfpv_indoor), which the runner reads to map channels to sensor ids.
Left and right frames of a stereo pair share a timestamp, so they land on the
same mcap logTime and the runner pairs them exactly.

Input layout (…/indoor_forward_<n>_snapdragon_with_gt):
  imu.txt           # id timestamp ang_vel_xyz lin_acc_xyz
  left_images.txt   # id timestamp image_name    (image_name relative to folder)
  right_images.txt  # id timestamp image_name
  groundtruth.txt   # timestamp tx ty tz qx qy qz qw   (optional)
  img/…             PNG frames

Usage: uzh_fpv_folder_to_mcap.py <sequence_dir> <output.mcap>
"""
import os
import sys

from mcap.writer import CompressionType, Writer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from euroc_folder_to_mcap import (  # noqa: E402  reuse the ros1msg serializers
    COMPRESSED_IMAGE_DEF,
    IMU_DEF,
    POSE_STAMPED_DEF,
    ser_compressed_image,
    ser_imu,
    ser_pose_stamped,
)

IMU_TOPIC = "/snappy_imu"
CAM_TOPICS = ["/snappy_cam/stereo_l", "/snappy_cam/stereo_r"]


def to_ns(sec_str):
    return int(round(float(sec_str) * 1e9))


def read_rows(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append(line.split())
    return rows


def read_images(seq_dir, name):
    # rows: id timestamp image_name(relative)
    out = []
    for r in read_rows(os.path.join(seq_dir, name)):
        out.append((to_ns(r[1]), os.path.join(seq_dir, r[2])))
    return out


def main():
    seq_dir, out_path = sys.argv[1], sys.argv[2]

    events = []  # (t_ns, order, kind, payload)

    imu_rows = read_rows(os.path.join(seq_dir, "imu.txt"))
    for r in imu_rows:
        t = to_ns(r[1])
        wx, wy, wz, ax, ay, az = (float(v) for v in r[2:8])
        events.append((t, 0, "imu", (wx, wy, wz, ax, ay, az)))

    left = read_images(seq_dir, "left_images.txt")
    right = read_images(seq_dir, "right_images.txt")
    for t, path in left:
        events.append((t, 2, "cam0", path))
    for t, path in right:
        events.append((t, 2, "cam1", path))

    gt_rows = []
    gt_path = os.path.join(seq_dir, "groundtruth.txt")
    if os.path.exists(gt_path):
        for r in read_rows(gt_path):
            t = to_ns(r[0])
            px, py, pz, qx, qy, qz, qw = (float(v) for v in r[1:8])
            gt_rows.append((t, px, py, pz, qx, qy, qz, qw))
            events.append((t, 1, "gt", (px, py, pz, qx, qy, qz, qw)))

    events.sort(key=lambda e: (e[0], e[1]))
    print(f"events: {len(imu_rows)} imu, {len(gt_rows)} gt, "
          f"{len(left)} left, {len(right)} right images")

    with open(out_path, "wb") as f:
        w = Writer(f, compression=CompressionType.NONE)
        w.start(profile="ros1", library="open_vins uzh_fpv_folder_to_mcap")

        imu_schema = w.register_schema("sensor_msgs/Imu", "ros1msg", IMU_DEF.encode())
        img_schema = w.register_schema("sensor_msgs/CompressedImage", "ros1msg",
                                       COMPRESSED_IMAGE_DEF.encode())
        pose_schema = w.register_schema("geometry_msgs/PoseStamped", "ros1msg",
                                        POSE_STAMPED_DEF.encode())

        channels = {
            "imu": w.register_channel(IMU_TOPIC, "ros1", imu_schema),
            "cam0": w.register_channel(CAM_TOPICS[0], "ros1", img_schema),
            "cam1": w.register_channel(CAM_TOPICS[1], "ros1", img_schema),
            "gt": w.register_channel("/gt/pose", "ros1", pose_schema),
        }

        seqs = {k: 0 for k in channels}
        for t, _, kind, payload in events:
            seq = seqs[kind]
            seqs[kind] += 1
            if kind == "imu":
                data = ser_imu(seq, t, *payload)
            elif kind == "gt":
                data = ser_pose_stamped(seq, t, *payload)
            else:
                with open(payload, "rb") as img:
                    data = ser_compressed_image(seq, t, kind, "png", img.read())
            w.add_message(channels[kind], t, data, t, seq)
        w.finish()

    if gt_rows:
        gt_txt = os.path.splitext(out_path)[0] + "_gt.txt"
        with open(gt_txt, "w") as f:
            f.write("# timestamp(s) tx ty tz qx qy qz qw\n")
            for t, px, py, pz, qx, qy, qz, qw in gt_rows:
                f.write(f"{t/1e9:.9f} {px} {py} {pz} {qx} {qy} {qz} {qw}\n")
        print(f"wrote {gt_txt}")
    print(f"wrote {out_path} ({os.path.getsize(out_path)/1e6:.1f} MB)")


if __name__ == "__main__":
    main()

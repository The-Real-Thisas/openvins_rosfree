#!/usr/bin/env python3
"""Convert a EuRoC/TUM-VI style mav0 folder into a single MCAP file.

Output channels (ros1msg encoding, matching what `mcap convert` produces from
official ROS1 bags, so the C++ runner has one ingestion path):
  /imu0             sensor_msgs/Imu
  /cam<N>/image_raw sensor_msgs/CompressedImage  (PNG bytes embedded as-is)
  /gt/pose          geometry_msgs/PoseStamped    (from mocap0 or state gt csv)

Chunks are written uncompressed: the payload is already-compressed PNGs, and
this lets the C++ reader build without lz4/zstd.

Also writes a TUM-format ground-truth text file next to the mcap for ov_eval.

Usage: euroc_folder_to_mcap.py <mav0_dir> <output.mcap>
"""
import csv
import os
import struct
import sys

from mcap.writer import CompressionType, Writer

HEADER_DEF = """uint32 seq
time stamp
string frame_id
"""

SEP = "=" * 80

IMU_DEF = (
    """std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance

"""
    + SEP
    + "\nMSG: std_msgs/Header\n"
    + HEADER_DEF
    + "\n"
    + SEP
    + """
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w

"""
    + SEP
    + """
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
"""
)

COMPRESSED_IMAGE_DEF = (
    """std_msgs/Header header
string format
uint8[] data

"""
    + SEP
    + "\nMSG: std_msgs/Header\n"
    + HEADER_DEF
)

POSE_STAMPED_DEF = (
    """std_msgs/Header header
geometry_msgs/Pose pose

"""
    + SEP
    + "\nMSG: std_msgs/Header\n"
    + HEADER_DEF
    + "\n"
    + SEP
    + """
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation

"""
    + SEP
    + """
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z

"""
    + SEP
    + """
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
"""
)


def ser_header(seq, t_ns, frame_id):
    fid = frame_id.encode()
    return struct.pack("<IIII", seq, t_ns // 10**9, t_ns % 10**9, len(fid)) + fid


def ser_imu(seq, t_ns, wx, wy, wz, ax, ay, az):
    cov_unknown = [-1.0] + [0.0] * 8
    cov_zero = [0.0] * 9
    return (
        ser_header(seq, t_ns, "imu0")
        + struct.pack("<4d", 0.0, 0.0, 0.0, 1.0)
        + struct.pack("<9d", *cov_unknown)
        + struct.pack("<3d", wx, wy, wz)
        + struct.pack("<9d", *cov_zero)
        + struct.pack("<3d", ax, ay, az)
        + struct.pack("<9d", *cov_zero)
    )


def ser_compressed_image(seq, t_ns, frame_id, fmt, data):
    f = fmt.encode()
    return (
        ser_header(seq, t_ns, frame_id)
        + struct.pack("<I", len(f))
        + f
        + struct.pack("<I", len(data))
        + data
    )


def ser_pose_stamped(seq, t_ns, px, py, pz, qx, qy, qz, qw):
    return ser_header(seq, t_ns, "world") + struct.pack("<7d", px, py, pz, qx, qy, qz, qw)


def read_csv(path):
    rows = []
    with open(path) as f:
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            rows.append(row)
    return rows


def main():
    mav0, out_path = sys.argv[1], sys.argv[2]

    # collect events: (t_ns, order, kind, payload_tuple)
    events = []

    imu_rows = read_csv(os.path.join(mav0, "imu0", "data.csv"))
    for r in imu_rows:
        t = int(r[0])
        events.append((t, 0, "imu", tuple(float(v) for v in r[1:7])))

    cam_dirs = sorted(d for d in os.listdir(mav0) if d.startswith("cam"))
    for cam in cam_dirs:
        for r in read_csv(os.path.join(mav0, cam, "data.csv")):
            t = int(r[0])
            events.append((t, 2, cam, os.path.join(mav0, cam, "data", r[1])))

    gt_rows = []
    mocap_csv = os.path.join(mav0, "mocap0", "data.csv")
    if os.path.exists(mocap_csv):
        # EuRoC mocap: t, px, py, pz, qw, qx, qy, qz
        for r in read_csv(mocap_csv):
            t = int(r[0])
            px, py, pz, qw, qx, qy, qz = (float(v) for v in r[1:8])
            gt_rows.append((t, px, py, pz, qx, qy, qz, qw))
            events.append((t, 1, "gt", (px, py, pz, qx, qy, qz, qw)))

    events.sort(key=lambda e: (e[0], e[1]))
    print(f"events: {len(imu_rows)} imu, {len(gt_rows)} gt, "
          f"{sum(1 for e in events if e[2].startswith('cam'))} images ({len(cam_dirs)} cams)")

    with open(out_path, "wb") as f:
        w = Writer(f, compression=CompressionType.NONE)
        w.start(profile="ros1", library="open_vins euroc_folder_to_mcap")

        imu_schema = w.register_schema("sensor_msgs/Imu", "ros1msg", IMU_DEF.encode())
        img_schema = w.register_schema("sensor_msgs/CompressedImage", "ros1msg",
                                       COMPRESSED_IMAGE_DEF.encode())
        pose_schema = w.register_schema("geometry_msgs/PoseStamped", "ros1msg",
                                        POSE_STAMPED_DEF.encode())

        channels = {"imu": w.register_channel("/imu0", "ros1", imu_schema),
                    "gt": w.register_channel("/gt/pose", "ros1", pose_schema)}
        for cam in cam_dirs:
            channels[cam] = w.register_channel(f"/{cam}/image_raw", "ros1", img_schema)

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

    # TUM-format ground truth for ov_eval (time[s] tx ty tz qx qy qz qw)
    if gt_rows:
        gt_path = os.path.splitext(out_path)[0] + "_gt.txt"
        with open(gt_path, "w") as f:
            f.write("# timestamp(s) tx ty tz qx qy qz qw\n")
            for t, px, py, pz, qx, qy, qz, qw in gt_rows:
                f.write(f"{t/1e9:.9f} {px} {py} {pz} {qx} {qy} {qz} {qw}\n")
        print(f"wrote {gt_path}")
    print(f"wrote {out_path} ({os.path.getsize(out_path)/1e6:.1f} MB)")


if __name__ == "__main__":
    main()

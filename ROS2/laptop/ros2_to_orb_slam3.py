#!/usr/bin/env python3

import argparse
from pathlib import Path
import shutil
import csv
import cv2
import numpy as np
from tqdm import tqdm

# --- ROS-related imports ---
# These require a sourced ROS2 environment.
try:
    from rosbags.highlevel import AnyReader
    from cv_bridge import CvBridge
except ImportError:
    print("=" * 50)
    print("IMPORT ERROR:")
    print("Could not import 'rosbags' or 'cv_bridge'.")
    print("Please ensure you have installed the required packages and sourced your ROS2 environment.")
    print("  pip install rosbags 'py-cv-bridge>=2.0.0' opencv-python-headless numpy tqdm")
    print("  source /opt/ros/<your_distro>/setup.bash")
    print("=" * 50)
    exit(1)


def create_output_dirs(base_dir: Path, format_type: str, is_stereo: bool):
    """Creates the necessary directory structure for the chosen format."""
    print(f"Creating directory structure for '{format_type}' format at {base_dir}...")
    if base_dir.exists():
        print(f"Output directory {base_dir} already exists. It will be cleared.")
        shutil.rmtree(base_dir)

    if format_type == 'euroc':
        # Structure for EuRoC format (visual-inertial)
        cam0_dir = base_dir / 'mav0' / 'cam0' / 'data'
        imu0_dir = base_dir / 'mav0' / 'imu0'
        timestamps_dir = base_dir / 'EuRoC_TimeStamps'

        cam0_dir.mkdir(parents=True, exist_ok=True)
        imu0_dir.mkdir(parents=True, exist_ok=True)
        timestamps_dir.mkdir(parents=True, exist_ok=True)

        if is_stereo:
            cam1_dir = base_dir / 'mav0' / 'cam1' / 'data'
            cam1_dir.mkdir(parents=True, exist_ok=True)

    elif format_type == 'tum':
        # Structure for TUM format (RGB-D)
        rgb_dir = base_dir / 'rgb'
        depth_dir = base_dir / 'depth'

        rgb_dir.mkdir(parents=True, exist_ok=True)
        depth_dir.mkdir(parents=True, exist_ok=True)

    print("Directory structure created successfully.")


def process_as_euroc(bag_path: Path, out_dir: Path, imu_topic: str, image_left_topic: str, image_right_topic: str):
    """
    Processes the ROS2 bag and outputs data in the EuRoC MAV format.
    This format is required for Visual-Inertial SLAM.
    """
    is_stereo = bool(image_right_topic)
    create_output_dirs(out_dir, 'euroc', is_stereo)

    bridge = CvBridge()
    imu_data = []
    cam0_timestamps = []
    cam1_timestamps = []

    print(f"Processing bag file: {bag_path}")
    with AnyReader([bag_path]) as reader:
        # Filter for the specified topics
        connections = [c for c in reader.connections if c.topic in [imu_topic, image_left_topic, image_right_topic]]

        for connection, timestamp, rawdata in tqdm(reader.messages(connections=connections), total=reader.message_count,
                                                   desc="Processing Messages"):
            msg = reader.deserialize(rawdata, connection.msgtype)

            if connection.topic == imu_topic:
                # IMPORTANT: Coordinate Frame Transformation
                # ROS standard (REP-103) uses FLU (Forward, Left, Up) for the body frame.
                # ORB-SLAM3's convention for IMU is RDF (Right, Down, Forward), aligned with the camera.
                # We must transform the IMU data accordingly.

                # Angular velocity transformation
                w_x = -msg.angular_velocity.y
                w_y = -msg.angular_velocity.z
                w_z = msg.angular_velocity.x

                # Linear acceleration transformation
                a_x = -msg.linear_acceleration.y
                a_y = -msg.linear_acceleration.z
                a_z = msg.linear_acceleration.x

                imu_data.append([
                    timestamp,
                    w_x, w_y, w_z,
                    a_x, a_y, a_z
                ])

            elif connection.topic == image_left_topic:
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                image_filename = f"{timestamp}.png"
                cv2.imwrite(str(out_dir / 'mav0' / 'cam0' / 'data' / image_filename), cv_image)
                cam0_timestamps.append(timestamp)

            elif is_stereo and connection.topic == image_right_topic:
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                image_filename = f"{timestamp}.png"
                cv2.imwrite(str(out_dir / 'mav0' / 'cam1' / 'data' / image_filename), cv_image)
                cam1_timestamps.append(timestamp)

    # --- Write IMU data to CSV ---
    print("Writing IMU data...")
    imu_data.sort(key=lambda x: x[0])  # Sort by timestamp
    imu_csv_path = out_dir / 'mav0' / 'imu0' / 'data.csv'
    with imu_csv_path.open('w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['#timestamp [ns]', 'w_x [rad/s]', 'w_y [rad/s]', 'w_z [rad/s]', 'a_x [m/s^2]', 'a_y [m/s^2]',
                         'a_z [m/s^2]'])
        writer.writerows(imu_data)

    # --- Write timestamp files ---
    # ORB-SLAM3's EuRoC example uses a single timestamp file based on the left camera.
    print("Writing timestamp data...")
    cam0_timestamps.sort()
    ts_path = out_dir / 'EuRoC_TimeStamps' / 'timestamps.txt'
    with ts_path.open('w') as f:
        for ts in cam0_timestamps:
            f.write(f"{ts / 1e9:.6f}\n")  # Timestamps in seconds

    print(f"EuRoC data generation complete. Output at: {out_dir}")


def process_as_tum(bag_path: Path, out_dir: Path, rgb_topic: str, depth_topic: str):
    """
    Processes the ROS2 bag and outputs data in the TUM RGB-D format.
    This format is required for RGB-D SLAM.
    """
    create_output_dirs(out_dir, 'tum', is_stereo=False)

    bridge = CvBridge()
    rgb_metadata = []
    depth_metadata = []

    print(f"Processing bag file: {bag_path}")
    with AnyReader([bag_path]) as reader:
        connections = [c for c in reader.connections if c.topic in [rgb_topic, depth_topic]]

        for connection, timestamp, rawdata in tqdm(reader.messages(connections=connections), total=reader.message_count,
                                                   desc="Processing Messages"):
            msg = reader.deserialize(rawdata, connection.msgtype)

            # Timestamp in seconds for TUM format
            ts_sec = timestamp / 1e9

            if connection.topic == rgb_topic:
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                image_filename = f"{ts_sec:.6f}.png"
                image_path = out_dir / 'rgb' / image_filename
                cv2.imwrite(str(image_path), cv_image)
                rgb_metadata.append((ts_sec, f"rgb/{image_filename}"))

            elif connection.topic == depth_topic:
                # Depth images are typically 16-bit single channel
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                image_filename = f"{ts_sec:.6f}.png"
                image_path = out_dir / 'depth' / image_filename
                cv2.imwrite(str(image_path), cv_image)
                depth_metadata.append((ts_sec, f"depth/{image_filename}"))

    # --- Write rgb.txt and depth.txt ---
    print("Writing rgb.txt and depth.txt...")
    rgb_metadata.sort(key=lambda x: x[0])
    depth_metadata.sort(key=lambda x: x[0])

    with (out_dir / 'rgb.txt').open('w') as f:
        f.write("# rgb image timestamps and path\n")
        for ts, path in rgb_metadata:
            f.write(f"{ts:.6f} {path}\n")

    with (out_dir / 'depth.txt').open('w') as f:
        f.write("# depth image timestamps and path\n")
        for ts, path in depth_metadata:
            f.write(f"{ts:.6f} {path}\n")

    # --- Create associations.txt by finding the nearest depth image for each RGB image ---
    print("Generating associations.txt...")
    associations = []
    depth_timestamps = [d[0] for d in depth_metadata]

    for rgb_ts, rgb_path in tqdm(rgb_metadata, desc="Associating frames"):
        # Find the index of the closest depth timestamp
        time_diffs = np.abs(np.array(depth_timestamps) - rgb_ts)
        best_match_idx = np.argmin(time_diffs)

        # We can add a threshold to discard pairs that are too far apart in time
        if time_diffs[best_match_idx] < 0.02:  # e.g., 20ms threshold
            depth_ts, depth_path = depth_metadata[best_match_idx]
            associations.append(f"{rgb_ts:.6f} {rgb_path} {depth_ts:.6f} {depth_path}\n")

    with (out_dir / 'associations.txt').open('w') as f:
        f.write("# rgb_timestamp rgb_path depth_timestamp depth_path\n")
        f.writelines(associations)

    print(f"TUM data generation complete. Output at: {out_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert a ROS2 bag file to ORB-SLAM3 compatible dataset formats (EuRoC or TUM).",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument('--bag', type=Path, required=True, help="Path to the input ROS2 bag directory.")
    parser.add_argument('--output-dir', type=Path, required=True, help="Directory to save the formatted dataset.")
    parser.add_argument('--format', type=str, required=True, choices=['euroc', 'tum'], help="Target dataset format.")

    # --- EuRoC arguments ---
    euroc_group = parser.add_argument_group('EuRoC Format Arguments')
    euroc_group.add_argument('--imu-topic', type=str,
                             help="IMU topic name (e.g., /imu/data). Required for 'euroc' format.")
    euroc_group.add_argument('--image-left-topic', type=str,
                             help="Left/Monocular image topic name (e.g., /cam0/image_raw). Required for 'euroc' format.")
    euroc_group.add_argument('--image-right-topic', type=str,
                             help="Right image topic name for stereo (e.g., /cam1/image_raw). Optional for 'euroc' format.")

    # --- TUM arguments ---
    tum_group = parser.add_argument_group('TUM Format Arguments')
    tum_group.add_argument('--rgb-topic', type=str,
                           help="RGB image topic name (e.g., /camera/rgb/image_raw). Required for 'tum' format.")
    tum_group.add_argument('--depth-topic', type=str,
                           help="Depth image topic name (e.g., /camera/depth/image_raw). Required for 'tum' format.")

    args = parser.parse_args()

    if not args.bag.exists() or not args.bag.is_dir():
        print(f"Error: Bag path '{args.bag}' does not exist or is not a directory.")
        return

    if args.format == 'euroc':
        if not args.imu_topic or not args.image_left_topic:
            parser.error("--imu-topic and --image-left-topic are required for --format='euroc'")
        process_as_euroc(args.bag, args.output_dir, args.imu_topic, args.image_left_topic, args.image_right_topic)
    elif args.format == 'tum':
        if not args.rgb_topic or not args.depth_topic:
            parser.error("--rgb-topic and --depth-topic are required for --format='tum'")
        process_as_tum(args.bag, args.output_dir, args.rgb_topic, args.depth_topic)


if __name__ == '__main__':
    main()

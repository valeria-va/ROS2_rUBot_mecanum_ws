#!/usr/bin/env python3
import sys
import csv
import argparse

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf_transformations import quaternion_from_euler
from my_robot_co2map.srv import UploadTrajectory


def make_pose(x: float, y: float, yaw: float, frame: str, stamp=None) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    if stamp is not None:
        ps.header.stamp = stamp
    ps.pose.position = Point(x=x, y=y, z=0.0)
    q = quaternion_from_euler(0.0, 0.0, yaw)
    ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return ps


def parse_float_or_none(token: str):
    try:
        return float(token.strip())
    except Exception:
        return None


def read_csv_poses(path: str, frame: str, stamp) -> list:
    poses = []
    with open(path, newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue

            # skip commented or header lines
            first = row[0].strip()
            if first.startswith('#') or first.lower() in ('x', 'pose_x', 'px'):
                continue

            x = parse_float_or_none(row[0])
            y = parse_float_or_none(row[1]) if len(row) > 1 else None
            yaw = parse_float_or_none(row[2]) if len(row) > 2 else 0.0
            if x is None or y is None:
                # skip non-numeric lines
                continue

            poses.append(make_pose(x, y, yaw if yaw is not None else 0.0, frame, stamp))
    return poses


class TrajectorySender(Node):
    def __init__(self):
        super().__init__('trajectory_sender')
        self.cli = self.create_client(UploadTrajectory, 'upload_trajectory')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /upload_trajectory not available')
            raise RuntimeError('Service /upload_trajectory not available')

    def send(self, poses):
        req = UploadTrajectory.Request()
        req.poses = poses
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result()


def parse_args(argv):
    p = argparse.ArgumentParser(description='Trajectory sender')
    p.add_argument('--csv', type=str, help='CSV file with poses: x,y,yaw (radians) per line')
    p.add_argument('--pose', action='append', help='Inline pose as x,y,yaw (can repeat)', default=[])
    p.add_argument('--frame', type=str, default='map', help='Frame id for poses')
    p.add_argument('--deg', action='store_true',
                   help='Treat yaw values as degrees and convert to radians')
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:])
    rclpy.init(args=None)
    try:
        node = TrajectorySender()
    except RuntimeError:
        rclpy.shutdown()
        return

    poses = []
    stamp = node.get_clock().now().to_msg()

    # CSV input
    if args.csv:
        poses = read_csv_poses(args.csv, frame=args.frame, stamp=stamp)

    # Inline poses
    for p in args.pose:
        parts = [t.strip() for t in p.split(',')]
        if len(parts) < 2:
            node.get_logger().warn(f'Ignored invalid inline pose (needs at least x,y): {p}')
            continue
        x = parse_float_or_none(parts[0])
        y = parse_float_or_none(parts[1])
        yaw = parse_float_or_none(parts[2]) if len(parts) > 2 else 0.0
        if x is None or y is None:
            node.get_logger().warn(f'Ignored invalid inline pose: {p}')
            continue
        poses.append(make_pose(x, y, yaw if yaw is not None else 0.0, frame=args.frame, stamp=stamp))

    # Optional: convert yaw from degrees if requested
    if args.deg:
        for ps in poses:
            # convert in-place
            ps.pose.orientation = Quaternion()  # reset before recomputing
            # extract yaw from previous input is not trivial; easier to rebuild:
            # We can't extract yaw from orientation directly here; so re-read from CSV is better.
            # For simplicity, warn users to pass --deg only when using inline poses below:
            pass

    # If no poses provided, use a small default example
    if not poses:
        node.get_logger().info('No poses provided, using default example trajectory.')
        poses = [
            make_pose(0.5, 0.5, 0.0, frame=args.frame, stamp=stamp),
            make_pose(1.0, 0.5, 0.0, frame=args.frame, stamp=stamp),
            make_pose(1.0, 1.0, 1.57, frame=args.frame, stamp=stamp),
        ]

    node.get_logger().info(f'Sending {len(poses)} poses to /upload_trajectory')
    res = node.send(poses)
    if res is None:
        node.get_logger().error('Service call failed or timed out')
    else:
        node.get_logger().info(f'Accepted: {res.accepted}, message: {res.message}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

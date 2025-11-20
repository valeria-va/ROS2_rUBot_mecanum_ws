#!/usr/bin/env python3
import os
import time
import csv
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

from ens160_interfaces.msg import SensorData
from my_robot_co2map.srv import UploadTrajectory

# Defaults
DEFAULT_CSV = os.path.expanduser('~/eco2_trajectory_log.csv')
DEFAULT_SAMPLE_TIME = 3.0                 # seconds per mini-sample window
DEFAULT_READINGS_PER_POSE = 3             # how many mini-samples to take at each waypoint
DEFAULT_ARRIVAL_TOL = 0.10                # not used directly in Nav2 goal; kept for future
DEFAULT_NAV_ACTION = 'navigate_to_pose'
DEFAULT_CHANNELS = 6                      # number of sensor channels (angular sensors)
DEFAULT_SAMPLES_PER_CHANNEL = 7           # samples per channel within a single sensor message

class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')

        # Parameters
        self.declare_parameter('sensor_topic', 'ens160_fused_data')
        self.declare_parameter('csv_path', DEFAULT_CSV)
        self.declare_parameter('sample_time', DEFAULT_SAMPLE_TIME)
        self.declare_parameter('readings_per_pose', DEFAULT_READINGS_PER_POSE)
        self.declare_parameter('arrival_tol', DEFAULT_ARRIVAL_TOL)
        self.declare_parameter('nav_action_name', DEFAULT_NAV_ACTION)
        self.declare_parameter('channels', DEFAULT_CHANNELS)
        self.declare_parameter('samples_per_channel', DEFAULT_SAMPLES_PER_CHANNEL)

        self.sensor_topic = self.get_parameter('sensor_topic').get_parameter_value().string_value
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.sample_time = self.get_parameter('sample_time').get_parameter_value().double_value
        self.readings_per_pose = self.get_parameter('readings_per_pose').get_parameter_value().integer_value
        self.arrival_tol = self.get_parameter('arrival_tol').get_parameter_value().double_value
        self.nav_action_name = self.get_parameter('nav_action_name').get_parameter_value().string_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.samples_per_channel = self.get_parameter('samples_per_channel').get_parameter_value().integer_value

        # TF and map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_msg = None
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)

        # Preview publisher
        self.preview_pub = self.create_publisher(MarkerArray, '/trajectory_preview', 10)

        # Nav2 action client
        self._action_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        # Service to upload trajectory
        self.srv = self.create_service(UploadTrajectory, 'upload_trajectory', self.handle_upload)

        # Sensor buffer
        self.sensor_buffer = []  # list of tuples (timestamp_sec, flat_vals_list)
        self.create_subscription(SensorData, self.sensor_topic, self.sensor_cb, 50)

        # CSV init
        self.init_csv()

        self.get_logger().info('TrajectoryExecutor ready. Service: /upload_trajectory')

    def init_csv(self):
        # Header includes per-channel mean/std plus per-reading means (optional compact summary)
        header = ['arrival_time', 'pose_x', 'pose_y', 'yaw', 'readings_per_pose']
        for i in range(1, self.channels + 1):
            header.append(f'chan{i}_mean')
        for i in range(1, self.channels + 1):
            header.append(f'chan{i}_std')
        # Also store the per-reading means (flattened) for post-hoc analysis
        for r in range(1, self.readings_per_pose + 1):
            for i in range(1, self.channels + 1):
                header.append(f'reading{r}_chan{i}_mean')
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)

    def map_cb(self, msg: OccupancyGrid):
        if self.map_msg is None:
            self.map_msg = msg

    def sensor_cb(self, msg: SensorData):
        ts = self.get_clock().now().nanoseconds / 1e9
        vals = list(msg.sensor_readings)
        required = self.channels * self.samples_per_channel
        # Pad if message is shorter than expected
        if len(vals) < required and len(vals) > 0:
            # simple padding: repeat values until required length
            while len(vals) < required:
                vals.extend(vals)
        # If empty (no readings), store nothing
        if len(vals) == 0:
            return
        vals = vals[:required]
        self.sensor_buffer.append((ts, vals))
        # Cap buffer size
        if len(self.sensor_buffer) > 10000:
            self.sensor_buffer = self.sensor_buffer[-5000:]

    def handle_upload(self, request, response):
        poses = request.poses
        if len(poses) == 0:
            response.accepted = False
            response.message = 'Empty pose list'
            return response

        self.publish_preview(poses)

        invalid = []
        if self.map_msg is not None:
            for i, p in enumerate(poses):
                if not self.pose_in_free_map(p):
                    invalid.append(i)
        if invalid:
            response.accepted = False
            response.message = f'Invalid pose indices in occupied/out-of-bounds cells: {invalid}'
            return response

        response.accepted = True
        response.message = 'Trajectory accepted. Executing.'
        self.get_logger().info(f'Accepted trajectory with {len(poses)} waypoints. Starting execution.')
        # run execution asynchronously via timer callback once
        self._exec_poses = poses
        self.create_timer(0.1, self._execute_timer_cb)
        return response

    def publish_preview(self, poses):
        ma = MarkerArray()
        for i, p in enumerate(poses):
            m = Marker()
            m.header.frame_id = p.header.frame_id or 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'trajectory_preview'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = p.pose
            m.scale.x = 0.12
            m.scale.y = 0.12
            m.scale.z = 0.12
            m.color = ColorRGBA(r=0.0, g=0.7, b=1.0, a=0.9)
            ma.markers.append(m)
        self.preview_pub.publish(ma)

    def pose_in_free_map(self, pose_stamped: PoseStamped) -> bool:
        try:
            if self.map_msg is None:
                return True
            res = self.map_msg.info.resolution
            origin = self.map_msg.info.origin
            width = self.map_msg.info.width
            height = self.map_msg.info.height
            mx = int((pose_stamped.pose.position.x - origin.position.x) / res)
            my = int((pose_stamped.pose.position.y - origin.position.y) / res)
            if mx < 0 or my < 0 or mx >= width or my >= height:
                return False
            idx = my * width + mx
            val = self.map_msg.data[idx]
            return (val == 0)  # 0 = free, >0 = occupied, -1 = unknown
        except Exception:
            return True

    def _execute_timer_cb(self):
        # cancel timer by removing attribute and run once
        if hasattr(self, '_exec_started') and self._exec_started:
            return
        self._exec_started = True
        poses = getattr(self, '_exec_poses', [])
        for idx, p in enumerate(poses):
            self.get_logger().info(f'[{idx+1}/{len(poses)}] Sending goal to ({p.pose.position.x:.2f},{p.pose.position.y:.2f})')
            success = self.send_nav_goal(p)
            if not success:
                self.get_logger().warn(f'Could not reach pose {idx}. Skipping.')
                continue
            self.sample_and_log(p)
        self.get_logger().info('Trajectory execution completed.')

    def send_nav_goal(self, pose_stamped: PoseStamped) -> bool:
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped
        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return False

        get_result_future = goal_handle.get_result_async()
        start = time.time()
        while rclpy.ok():
            if get_result_future.done():
                status = get_result_future.result().status
                if status == 4:  # SUCCEEDED
                    return True
                else:
                    self.get_logger().warn(f'Nav2 finished with status {status}')
                    return False
            if time.time() - start > 30.0:
                self.get_logger().warn('Timeout navigating to goal; cancelling')
                try:
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                except Exception:
                    pass
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def sample_and_log(self, pose_stamped: PoseStamped):
        """
        Take self.readings_per_pose independent mini-samples at the waypoint.
        Each mini-sample collects sensor messages for self.sample_time seconds,
        reduces them to per-channel means/std, then we average across mini-samples.
        Also logs the per-reading means for later analysis.
        """
        per_reading_means = []  # list of arrays shape (channels,)
        per_reading_stds = []   # list of arrays shape (channels,)

        for r in range(self.readings_per_pose):
            t0 = self.get_clock().now().nanoseconds / 1e9
            samples = []
            start_wall = time.time()
            while time.time() - start_wall < self.sample_time:
                rclpy.spin_once(self, timeout_sec=0.1)
                # collect sensor messages that arrived after t0
                for ts, vals in list(self.sensor_buffer):
                    if ts >= t0:
                        samples.append(vals)

            if len(samples) == 0:
                means = [float('nan')] * self.channels
                stds = [float('nan')] * self.channels
            else:
                arr = np.array(samples, dtype=float)  # shape: (n_samples, channels*samples_per_channel)
                # Reshape into [n_samples, channels, samples_per_channel]
                try:
                    arr = arr.reshape((-1, self.channels, self.samples_per_channel))
                    # mean across inner per-channel samples
                    channel_means_per_msg = np.nanmean(arr, axis=2)  # shape: (n_samples, channels)
                    # aggregate across messages in this mini-sample window
                    means = list(np.nanmean(channel_means_per_msg, axis=0))
                    stds = list(np.nanstd(channel_means_per_msg, axis=0))
                except ValueError:
                    # Fallback if shape mismatch
                    # Split flat list into channels (approximate)
                    flat = np.array(samples, dtype=float)
                    # average across all collected frames
                    means = []
                    stds = []
                    for ch in range(self.channels):
                        ch_vals = flat[:, ch::self.channels].flatten()
                        means.append(float(np.nanmean(ch_vals)) if ch_vals.size else float('nan'))
                        stds.append(float(np.nanstd(ch_vals)) if ch_vals.size else float('nan'))

            per_reading_means.append(np.array(means, dtype=float))
            per_reading_stds.append(np.array(stds, dtype=float))

        # Average the mini-samples to produce final values for this waypoint
        all_means = np.nanmean(np.stack(per_reading_means, axis=0), axis=0)  # shape: (channels,)
        all_stds = np.nanmean(np.stack(per_reading_stds, axis=0), axis=0)    # shape: (channels,)

        yaw = self.get_yaw_from_pose(pose_stamped.pose)
        arrival_time = time.time()
        row = [arrival_time, pose_stamped.pose.position.x, pose_stamped.pose.position.y, yaw, self.readings_per_pose]
        row += list(all_means)
        row += list(all_stds)
        # Append per-reading channel means for transparency
        for r in range(self.readings_per_pose):
            row += list(per_reading_means[r])

        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
        self.get_logger().info(f'Sampled {self.readings_per_pose}x at waypoint; means={list(np.round(all_means, 2))}')

    def get_yaw_from_pose(self, pose) -> float:
        q = pose.orientation
        try:
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        except Exception:
            yaw = 0.0
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

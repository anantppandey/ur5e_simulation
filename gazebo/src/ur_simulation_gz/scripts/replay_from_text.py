#!/usr/bin/env python3
"""
Replay trajectory from a text file by sending FollowJointTrajectory action.
Input format (comment lines starting with # are ignored):
  time joint1 joint2 joint3 joint4 joint5 joint6

Usage:
  python3 replay_from_text.py <joints.txt> [--controller scaled_joint_trajectory_controller] [--joint-names j1 j2 ...]
"""
import sys
import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


DEFAULT_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]


def load_text_trajectory(path):
    times = []
    positions = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            t = float(parts[0])
            pos = [float(p) for p in parts[1:]]
            times.append(t)
            positions.append(pos)
    return times, positions


class ReplayNode(Node):
    def __init__(self, controller_action_name):
        super().__init__('replay_from_text')
        self._action_client = ActionClient(self, FollowJointTrajectory, controller_action_name)

    def wait_for_action_server(self, timeout=5.0):
        return self._action_client.wait_for_server(timeout_sec=timeout)

    def send_trajectory(self, joint_names, times, positions, move_to_start_duration=3.0):
        # First move to the starting position
        self.get_logger().info(f'Moving to start position: {positions[0]}')
        start_goal = FollowJointTrajectory.Goal()
        start_traj = JointTrajectory()
        start_traj.joint_names = joint_names
        
        start_point = JointTrajectoryPoint()
        start_point.positions = positions[0]
        start_point.time_from_start = Duration(sec=int(move_to_start_duration), nanosec=0)
        start_traj.points = [start_point]
        start_goal.trajectory = start_traj
        
        send_goal_future = self._action_client.send_goal_async(start_goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Move-to-start goal rejected')
            return None
        
        self.get_logger().info('Move-to-start accepted, waiting...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        start_result = result_future.result()
        
        if start_result.status != 4:  # SUCCESS
            self.get_logger().warn(f'Move-to-start finished with status: {start_result.status}')
        
        # Now send the full trajectory
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = joint_names

        points = []
        t0 = times[0]
        for t, pos in zip(times, positions):
            p = JointTrajectoryPoint()
            p.positions = pos
            dt = t - t0
            p.time_from_start = Duration(sec=int(dt), nanosec=int((dt - int(dt)) * 1e9))
            points.append(p)

        traj.points = points
        goal.trajectory = traj

        self.get_logger().info(f'Sending main trajectory with {len(points)} points')
        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return None

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('textfile', help='Text file with trajectory')
    parser.add_argument('--controller', default='scaled_joint_trajectory_controller', help='Controller name')
    parser.add_argument('--joint-names', nargs='+', default=DEFAULT_JOINT_NAMES, help='Joint names in order')
    parser.add_argument('--downsample', type=int, default=10, help='Downsample factor (use every Nth point, default: 10)')
    args = parser.parse_args()

    times, positions = load_text_trajectory(args.textfile)
    
    # Downsample to reduce number of points
    if args.downsample > 1:
        times = times[::args.downsample]
        positions = positions[::args.downsample]
        print(f'Downsampled to {len(times)} points (factor: {args.downsample})')
    
    n_joints = len(positions[0])
    if len(args.joint_names) != n_joints:
        print(f'Error: text file has {n_joints} joints but {len(args.joint_names)} joint names provided')
        return

    rclpy.init()
    node = ReplayNode(f'/{args.controller}/follow_joint_trajectory')
    if not node.wait_for_action_server(timeout=5.0):
        node.get_logger().error('Action server not available')
        return

    result = node.send_trajectory(args.joint_names, times, positions)
    if result:
        node.get_logger().info(f'Trajectory result status: {result.status}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

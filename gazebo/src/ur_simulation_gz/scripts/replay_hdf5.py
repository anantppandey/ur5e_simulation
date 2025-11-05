#!/usr/bin/env python3
"""
Replay a trajectory stored in an HDF5 file by sending FollowJointTrajectory goals.
Expected HDF5 layout (simple):
  /joint_names (dataset of strings) or attribute
  /time (1D dataset in seconds)
  /positions (2D dataset: len(time) x len(joint_names))
Optional:
  /velocities (2D same shape)
  /efforts (2D same shape)
  /sensors/<topic_name> (datasets aligned with time)

Usage:
  python3 replay_hdf5.py /path/to/file.h5 --controller scaled_joint_trajectory_controller --rate 50

This script will build a JointTrajectory with points sampled from the HDF5 and send it as a single goal.
"""

import argparse
import time as pytime
import numpy as np
import h5py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def load_hdf5(path):
    with h5py.File(path, 'r') as f:
        # joint names
        if 'joint_names' in f:
            joint_names = [n.decode('utf-8') if isinstance(n, bytes) else str(n) for n in f['joint_names'][()]]
        elif 'joint_names' in f.attrs:
            joint_names = [n.decode('utf-8') if isinstance(n, bytes) else str(n) for n in f.attrs['joint_names']]
        else:
            raise RuntimeError('HDF5 must contain /joint_names dataset or attribute')

        time_vec = f['time'][()]
        positions = f['positions'][()]
        velocities = f['velocities'][()] if 'velocities' in f else None
        efforts = f['efforts'][()] if 'efforts' in f else None

    return joint_names, time_vec, positions, velocities, efforts


class ReplayNode(Node):
    def __init__(self, controller_action_name):
        super().__init__('replay_hdf5')
        self._action_client = ActionClient(self, FollowJointTrajectory, controller_action_name)

    def wait_for_action_server(self, timeout=5.0):
        return self._action_client.wait_for_server(timeout_sec=timeout)

    def send_trajectory(self, joint_names, time_vec, positions):
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = joint_names

        points = []
        t0 = time_vec[0]
        for t, pos in zip(time_vec, positions):
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in pos]
            dt = float(t - t0)
            # time_from_start relative to beginning
            p.time_from_start = Duration(sec=int(dt), nanosec=int((dt - int(dt)) * 1e9))
            points.append(p)

        traj.points = points
        goal.trajectory = traj

        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return None

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('h5file', help='HDF5 file path')
    parser.add_argument('--controller', default='scaled_joint_trajectory_controller', help='Controller name')
    parser.add_argument('--rate', type=int, default=50, help='Sample rate for constructing trajectory (Hz)')
    parser.add_argument('--dry-run', action='store_true', help='Do not send trajectory; just print file summary and exit')
    args = parser.parse_args()

    joint_names, time_vec, positions, velocities, efforts = load_hdf5(args.h5file)

    # Dry-run: print summary and exit without contacting ROS
    if args.dry_run:
        print('HDF5 summary for:', args.h5file)
        print('  joints:', joint_names)
        print(f'  time: start={time_vec[0]:.6f}s end={time_vec[-1]:.6f}s points={len(time_vec)}')
        print('  positions shape:', np.shape(positions))
        if velocities is not None:
            print('  velocities shape:', np.shape(velocities))
        if efforts is not None:
            print('  efforts shape:', np.shape(efforts))
        return

    rclpy.init()
    node = ReplayNode(f'/{args.controller}/follow_joint_trajectory')
    if not node.wait_for_action_server(timeout=5.0):
        node.get_logger().error('Action server not available')
        return

    node.get_logger().info(f'Sending trajectory with {len(time_vec)} points for joints: {joint_names}')
    res = node.send_trajectory(joint_names, time_vec, positions)
    node.get_logger().info(f'Trajectory result: {res}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

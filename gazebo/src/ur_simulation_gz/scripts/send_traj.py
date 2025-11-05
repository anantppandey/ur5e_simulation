#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

NODE_NAME = 'send_traj_goal'
ACTION_NAME = '/scaled_joint_trajectory_controller/follow_joint_trajectory'

JOINTS = [
    "shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
    "wrist_1_joint","wrist_2_joint","wrist_3_joint"
]


def main():
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)
    client = ActionClient(node, FollowJointTrajectory, ACTION_NAME)

    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error(f"Action server {ACTION_NAME} not available")
        node.destroy_node()
        rclpy.shutdown()
        return

    goal = FollowJointTrajectory.Goal()
    traj = JointTrajectory()
    traj.joint_names = JOINTS

    p = JointTrajectoryPoint()
    p.positions = [0.0, -1.0, 0.5, -1.0, 0.0, 0.0]
    p.time_from_start = Duration(sec=3, nanosec=0)
    traj.points = [p]

    goal.trajectory = traj

    send_goal_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        node.get_logger().error("Goal rejected")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("Goal accepted, waiting for result...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()
    node.get_logger().info(f"Result: {result.status}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

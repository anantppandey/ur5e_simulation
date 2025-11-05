#!/bin/bash

# Helper script to reload controllers when they get stuck

echo "Unloading scaled_joint_trajectory_controller..."
ros2 control unload_controller scaled_joint_trajectory_controller

echo "Waiting 2 seconds..."
sleep 2

echo "Loading scaled_joint_trajectory_controller..."
ros2 control load_controller scaled_joint_trajectory_controller

echo "Waiting 2 seconds..."
sleep 2

echo "Configuring scaled_joint_trajectory_controller..."
ros2 control set_controller_state scaled_joint_trajectory_controller configure

echo "Waiting 2 seconds..."
sleep 2

echo "Activating scaled_joint_trajectory_controller..."
ros2 control set_controller_state scaled_joint_trajectory_controller activate

echo "Done! Controller should be active now."
echo "Check status with: ros2 control list_controllers"

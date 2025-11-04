import numpy as np
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.prims import get_prim_at_path
import omni.kit.app
import asyncio
from pxr import Usd

# Global variable to prevent multiple simultaneous runs
_is_running = False
_should_stop = False

def map_gripper_value(gripper_value):
    """
    Map gripper value from file format to robot format.
    
    Args:
        gripper_value: Gripper value from file (range 0.0075 to 0.08)
        
    Returns:
        Mapped gripper value (range 0.675 to 0.0)
        
    Mapping:
        0.08 (file) -> 0.0 (robot, open)
        0.0075 (file) -> 0.675 (robot, closed)
    """
    input_min = 0.0075
    input_max = 0.08
    output_min = 0.675
    output_max = 0.0
    
    gripper_value = max(input_min, min(input_max, gripper_value))
    normalized = (gripper_value - input_min) / (input_max - input_min)
    mapped_value = output_min + normalized * (output_max - output_min)
    
    return mapped_value

def moving_average_filter(trajectory, gripper_trajectory, window_size):
    """
    Apply moving average filter to smooth trajectory while maintaining same number of points.
    
    Args:
        trajectory: List of numpy arrays containing joint positions
        gripper_trajectory: List of gripper positions
        window_size: Size of the moving average window (must be odd)
        
    Returns:
        Tuple of (smoothed_trajectory, smoothed_gripper)
    """
    if window_size < 1:
        return trajectory, gripper_trajectory
    
    # Ensure window size is odd
    if window_size % 2 == 0:
        window_size += 1
    
    half_window = window_size // 2
    
    smoothed_trajectory = []
    smoothed_gripper = []
    
    for i in range(len(trajectory)):
        # Determine the range for averaging
        start_idx = max(0, i - half_window)
        end_idx = min(len(trajectory), i + half_window + 1)
        
        # Average joint positions
        avg_joints = np.mean([trajectory[j] for j in range(start_idx, end_idx)], axis=0)
        smoothed_trajectory.append(avg_joints)
        
        # Average gripper positions
        avg_gripper = np.mean([gripper_trajectory[j] for j in range(start_idx, end_idx)])
        smoothed_gripper.append(avg_gripper)
    
    print(f"Applied moving average filter with window size {window_size}")
    print(f"Original trajectory points: {len(trajectory)}, Smoothed: {len(smoothed_trajectory)}")
    
    return smoothed_trajectory, smoothed_gripper

def load_trajectory_from_file(filepath):
    """
    Load trajectory from a text file including gripper values.
    """
    joint_trajectory = []
    gripper_trajectory = []
    
    try:
        with open(filepath, 'r') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                
                if not line or line.startswith('#'):
                    continue
                
                values = line.split()
                
                if len(values) < 8:
                    print(f"Warning: Line {line_num} has only {len(values)} values, expected 8. Skipping.")
                    continue
                
                try:
                    joint_positions = np.array([float(v) for v in values[1:7]])
                    gripper_position = float(values[7])
                    joint_trajectory.append(joint_positions)
                    gripper_trajectory.append(gripper_position)
                except ValueError as e:
                    print(f"Warning: Could not parse line {line_num}: {e}. Skipping.")
                    continue
        
        print(f"Loaded {len(joint_trajectory)} trajectory points from {filepath}")
        print(f"Gripper range (file format): [{min(gripper_trajectory):.6f}, {max(gripper_trajectory):.6f}]")
        
        mapped_gripper_trajectory = [map_gripper_value(g) for g in gripper_trajectory]
        print(f"Gripper range (mapped to robot): [{min(mapped_gripper_trajectory):.6f}, {max(mapped_gripper_trajectory):.6f}]")
        
        return joint_trajectory, mapped_gripper_trajectory
        
    except FileNotFoundError:
        print(f"Error: File not found: {filepath}")
        return None, None
    except Exception as e:
        print(f"Error loading trajectory file: {e}")
        return None, None

def find_robot_prim_path(robot_name="ur5e"):
    """Find the actual prim path of the robot in the stage"""
    from omni.isaac.core.utils.stage import get_current_stage
    
    stage = get_current_stage()
    possible_paths = [
        f"/{robot_name}",
        f"/World/{robot_name}",
        f"/UR5e",
        f"/World/UR5e",
        f"/ur5e",
        f"/World/ur5e",
    ]
    
    for path in possible_paths:
        prim = get_prim_at_path(path)
        if prim and prim.IsValid():
            print(f"Found robot at: {path}")
            return path
    
    print("Searching stage for UR5e robot...")
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        prim_name = prim.GetName().lower()
        if "ur5e" in prim_name or "ur5" in prim_name:
            path = str(prim.GetPath())
            print(f"Found possible robot at: {path}")
            return path
    
    return None

def stop_trajectory():
    """
    Stop the currently running trajectory execution.
    """
    global _should_stop
    _should_stop = True
    print("Stop signal sent. Trajectory will stop at next checkpoint...")

async def cleanup_and_stop():
    """
    Stop trajectory and completely stop simulation.
    """
    global _is_running, _should_stop
    
    print("Cleaning up and stopping...")
    _should_stop = True
    
    await asyncio.sleep(0.5)
    
    my_world = World.instance()
    if my_world is not None:
        if my_world.is_playing():
            print("Stopping simulation completely...")
            await my_world.stop_async()
        
        print("Clearing World instance...")
        World.clear_instance()
        print("Simulation stopped.")
    
    _is_running = False
    _should_stop = False
    print("Cleanup complete!")

async def reset_scene():
    """
    Reset the scene to initial state.
    """
    global _is_running, _should_stop
    
    print("Resetting scene...")
    _should_stop = True
    _is_running = False
    
    await asyncio.sleep(0.5)
    
    my_world = World.instance()
    if my_world is not None:
        print("Stopping simulation...")
        if my_world.is_playing():
            await my_world.stop_async()
        
        print("Clearing World instance...")
        World.clear_instance()
        print("Scene reset complete!")
    else:
        print("No world instance found.")
    
    _should_stop = False
    print("Ready for new trajectory!")

async def execute_trajectory(joint_trajectory, gripper_trajectory, steps_per_point, robot_name="ur5e"):
    """
    Execute a trajectory on the UR robot with gripper control.
    
    Args:
        joint_trajectory: List of joint positions
        gripper_trajectory: List of gripper positions
        steps_per_point: Simulation steps per trajectory point
        robot_name: Name of the robot
    """
    global _is_running, _should_stop
    
    if _is_running:
        print("Already running! Please wait or call stop_trajectory() to stop it.")
        return
    
    _is_running = True
    _should_stop = False
    
    try:
        my_world = World.instance()
        if my_world is None:
            my_world = World(stage_units_in_meters=1.0)
            await my_world.initialize_simulation_context_async()
            print("Created new World instance")
        
        my_robot = my_world.scene.get_object(robot_name)
        
        if my_robot is None:
            print(f"Robot '{robot_name}' not in scene, searching for it...")
            robot_prim_path = find_robot_prim_path(robot_name)
            
            if robot_prim_path is None:
                print(f"Error: Could not find UR5e robot in the stage.")
                return
            
            print(f"Registering robot from {robot_prim_path}...")
            my_robot = Robot(prim_path=robot_prim_path, name=robot_name)
            my_world.scene.add(my_robot)
            await my_world.reset_async()
            print(f"Robot '{robot_name}' registered successfully")
        
        if not my_world.is_playing():
            await my_world.play_async()
        
        my_robot.initialize()
        print("Initializing physics simulation view...")
        for i in range(50):
            if _should_stop:
                print("Stopped during initialization")
                await my_world.stop_async()
                World.clear_instance()
                return
            my_world.step(render=True)
            if i % 10 == 0:
                await omni.kit.app.get_app().next_update_async()
        
        current_pos = None
        for attempt in range(10):
            if _should_stop:
                print("Stopped during position reading")
                await my_world.stop_async()
                World.clear_instance()
                return
            current_pos = my_robot.get_joint_positions()
            if current_pos is not None:
                break
            print(f"Waiting for joint positions... (attempt {attempt + 1}/10)")
            my_world.step(render=True)
            await omni.kit.app.get_app().next_update_async()
        
        if current_pos is None:
            print("Error: Could not read joint positions after multiple attempts")
            await my_world.stop_async()
            World.clear_instance()
            return
        
        num_joints = len(current_pos)
        print(f"Robot has {num_joints} joints")
        print(f"Starting position: {np.round(current_pos[:6], 3)}")
        if num_joints > 6:
            print(f"Starting gripper position: {current_pos[6]:.6f}")
        print(f"Executing trajectory with {len(joint_trajectory)} points...")
        print(f"Steps per point: {steps_per_point}")
        print(f"Effective frequency: {50.0/steps_per_point:.2f} Hz")
        print(f"Estimated duration: {len(joint_trajectory) * steps_per_point / 50.0:.2f} seconds")
        print("Call stop_trajectory() to stop execution\n")
        
        articulation_controller = my_robot.get_articulation_controller()
        
        import time
        start_time = time.time()
        
        total_steps = 0
        point_idx = 0
        batch_size = 10
        
        while point_idx < len(joint_trajectory):
            if _should_stop:
                print(f"Trajectory stopped at point {point_idx}/{len(joint_trajectory)}")
                break
            
            points_in_batch = min(batch_size, len(joint_trajectory) - point_idx)
            
            for batch_idx in range(points_in_batch):
                if _should_stop:
                    break
                    
                current_point_idx = point_idx + batch_idx
                
                target_joints = joint_trajectory[current_point_idx]
                target_gripper = gripper_trajectory[current_point_idx]
                
                target_joint_positions = current_pos.copy()
                target_joint_positions[:6] = target_joints
                
                if num_joints > 6:
                    target_joint_positions[6] = target_gripper
                
                action = ArticulationAction(joint_positions=target_joint_positions)
                articulation_controller.apply_action(action)
                
                for step in range(steps_per_point):
                    if _should_stop:
                        break
                    my_world.step(render=True)
                    total_steps += 1
            
            point_idx += points_in_batch
            await asyncio.sleep(0)
            
            if point_idx % 100 == 0 or point_idx >= len(joint_trajectory):
                elapsed_time = time.time() - start_time
                actual_hz = total_steps / elapsed_time if elapsed_time > 0 else 0
                print(f"Progress: {point_idx}/{len(joint_trajectory)} ({100*point_idx//len(joint_trajectory)}%) - Elapsed: {elapsed_time:.2f}s - Hz: {actual_hz:.2f}")
        
        elapsed = time.time() - start_time
        final_pos = my_robot.get_joint_positions()
        
        if not _should_stop:
            print(f"\nFinal position: {np.round(final_pos[:6], 3)}")
            if num_joints > 6:
                print(f"Final gripper position: {final_pos[6]:.6f}")
            print(f"Trajectory execution complete!")
        
        print(f"Total time: {elapsed:.2f}s")
        print(f"Total simulation steps: {total_steps}")
        print(f"Actual simulation Hz: {total_steps/elapsed:.2f}")
        print(f"Expected: {50.0/steps_per_point:.2f} Hz")
        
        print("\n=== AUTO-CLEANUP ===")
        if my_world.is_playing():
            print("Stopping simulation completely...")
            await my_world.stop_async()
        
        print("Clearing World instance...")
        World.clear_instance()
        print("Simulation stopped.")
        print("Safe to use GUI now. Run script again to execute another trajectory.")
        print("===================\n")
        
    except Exception as e:
        print(f"Error during trajectory execution: {e}")
        import traceback
        traceback.print_exc()
        my_world = World.instance()
        if my_world is not None:
            if my_world.is_playing():
                await my_world.stop_async()
            World.clear_instance()
    finally:
        _is_running = False
        _should_stop = False
        print("Trajectory execution ended. Simulation stopped. Safe to use GUI.")

# Load trajectory from file
trajectory_file = "/home/azureuser/IsaacSim/projects/episode_0_full.txt"
print(f"Loading trajectory from: {trajectory_file}")
original_joints, original_gripper = load_trajectory_from_file(trajectory_file)

if original_joints is not None and len(original_joints) > 0:
    print(f"Successfully loaded {len(original_joints)} trajectory points")
    
    # ==================== MANUAL VARIABLES - CHANGE THESE ====================
    start_point = 0  # Change this: 0-based index (0 to 3391)
    end_point = 3391  # Change this: 0-based index (0 to 3391)
    
    target_duration_seconds = 100  # Change this to control speed
    
    # Smoothing filter window size (must be odd, 0 = no smoothing)
    smoothing_window_size = 5  # Change this: 0 for no smoothing, 3-11 for smoothing
    # =========================================================================
    
    full_trajectory_points = 3392
    steps_per_point = int(round((target_duration_seconds * 50.0) / full_trajectory_points))
    
    if steps_per_point < 1:
        steps_per_point = 1
        print(f"Warning: Target duration too short, using minimum steps_per_point = 1")
    
    print(f"Target duration for {full_trajectory_points} points: {target_duration_seconds}s")
    print(f"Calculated steps_per_point: {steps_per_point}")
    print(f"Actual duration for full trajectory: {full_trajectory_points * steps_per_point / 50.0:.2f}s")
    print(f"Effective frequency: {50.0/steps_per_point:.2f} Hz")
    
    if start_point < 0:
        start_point = 0
    if end_point >= len(original_joints):
        end_point = len(original_joints) - 1
    if start_point > end_point:
        print("Error: start_point must be <= end_point")
        exit(1)
    
    # Slice the trajectory
    selected_joints = original_joints[start_point:end_point+1]
    selected_gripper = original_gripper[start_point:end_point+1]
    
    # Apply smoothing filter if window size > 0
    if smoothing_window_size > 0:
        print(f"Applying moving average filter with window size {smoothing_window_size}...")
        selected_joints, selected_gripper = moving_average_filter(
            selected_joints, 
            selected_gripper, 
            smoothing_window_size
        )
    else:
        print("No smoothing applied (smoothing_window_size = 0)")
    
    print(f"Running trajectory from point {start_point} to {end_point} ({len(selected_joints)} points)")
    print(f"Estimated duration for selected range: {len(selected_joints) * steps_per_point / 50.0:.2f} seconds")
    print("\n=== CONTROL FUNCTIONS ===")
    print("To STOP trajectory: stop_trajectory()")
    print("To STOP and cleanup: asyncio.ensure_future(cleanup_and_stop())")
    print("To RESET scene: asyncio.ensure_future(reset_scene())")
    print("========================\n")
    
    asyncio.ensure_future(execute_trajectory(
        selected_joints, 
        selected_gripper, 
        steps_per_point, 
        robot_name="ur5e"
    ))
else:
    print("Failed to load trajectory.")
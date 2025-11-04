import numpy as np
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.prims import get_prim_at_path
import omni.kit.app
import asyncio
from pxr import Usd
import os
from PIL import Image
import omni.replicator.core as rep

# Global variable to prevent multiple simultaneous runs
_is_running = False

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
    # Input range: [0.0075, 0.08]
    # Output range: [0.675, 0.0]
    
    input_min = 0.0075
    input_max = 0.08
    output_min = 0.675
    output_max = 0.0
    
    # Clamp input value to valid range
    gripper_value = max(input_min, min(input_max, gripper_value))
    
    # Linear mapping
    normalized = (gripper_value - input_min) / (input_max - input_min)
    mapped_value = output_min + normalized * (output_max - output_min)
    
    return mapped_value

def load_trajectory_from_file(filepath):
    """
    Load trajectory from a text file including gripper values.
    
    Args:
        filepath: Path to the trajectory file
        
    Returns:
        Tuple of (joint_trajectory, gripper_trajectory)
        - joint_trajectory: List of numpy arrays, each containing 6 joint positions
        - gripper_trajectory: List of gripper position values
        
    Expected file format:
        - Lines starting with # are comments
        - Data lines: time joint1 joint2 joint3 joint4 joint5 joint6 gripper
    """
    joint_trajectory = []
    gripper_trajectory = []
    
    try:
        with open(filepath, 'r') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                
                # Skip empty lines or comments
                if not line or line.startswith('#'):
                    continue
                
                # Parse the line - split by whitespace
                values = line.split()
                
                if len(values) < 8:
                    print(f"Warning: Line {line_num} has only {len(values)} values, expected 8 (time + 6 joints + gripper). Skipping.")
                    continue
                
                # Convert to float - skip first value (time), take next 6 (joint positions), then gripper
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
        
        # Map gripper values to robot format
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
    
    # If not found, search the entire stage
    print("Searching stage for UR5e robot...")
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        prim_name = prim.GetName().lower()
        if "ur5e" in prim_name or "ur5" in prim_name:
            path = str(prim.GetPath())
            print(f"Found possible robot at: {path}")
            return path
    
    return None

def find_camera_prim_path(camera_name="wrist_0_camera"):
    """Find the camera prim path in the stage"""
    from omni.isaac.core.utils.stage import get_current_stage
    
    stage = get_current_stage()
    
    # Search the entire stage for the camera
    print(f"Searching for camera: {camera_name}")
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        prim_name = prim.GetName()
        if camera_name in prim_name:
            path = str(prim.GetPath())
            print(f"Found camera at: {path}")
            return path
    
    print(f"Warning: Could not find camera '{camera_name}'")
    return None

async def execute_trajectory_with_camera(joint_trajectory, gripper_trajectory, robot_name="ur5e", output_dir="/home/azureuser/IsaacSim/projects"):
    """
    Execute a trajectory on the UR robot with gripper control and save camera images.
    
    Args:
        joint_trajectory: List of numpy arrays, each containing 6 joint positions
        gripper_trajectory: List of gripper position values (already mapped)
        robot_name: Name of the robot in the scene (default: "ur5e")
        output_dir: Directory to save camera images
    """
    global _is_running
    
    if _is_running:
        print("Already running! Please wait...")
        return
    
    _is_running = True
    
    try:
        # Create output directory for camera images
        camera_output_dir = os.path.join(output_dir, "wrist_0_camera")
        os.makedirs(camera_output_dir, exist_ok=True)
        print(f"Camera images will be saved to: {camera_output_dir}")
        
        # Get or create World instance
        my_world = World.instance()
        if my_world is None:
            my_world = World(stage_units_in_meters=1.0)
            await my_world.initialize_simulation_context_async()
            print("Created new World instance")
        
        # Get the robot by name, or register it if not in scene
        my_robot = my_world.scene.get_object(robot_name)
        
        if my_robot is None:
            print(f"Robot '{robot_name}' not in scene, searching for it...")
            robot_prim_path = find_robot_prim_path(robot_name)
            
            if robot_prim_path is None:
                print(f"Error: Could not find UR5e robot in the stage. Please load it first through GUI.")
                return
            
            print(f"Registering robot from {robot_prim_path}...")
            my_robot = Robot(prim_path=robot_prim_path, name=robot_name)
            my_world.scene.add(my_robot)
            await my_world.reset_async()
            print(f"Robot '{robot_name}' registered successfully")
        
        # Find the wrist camera
        camera_prim_path = find_camera_prim_path("wrist_0_camera")
        if camera_prim_path is None:
            print("Error: Could not find wrist_0_camera in the scene")
            return
        
        # Create render product for the existing camera using replicator
        render_product = rep.create.render_product(camera_prim_path, resolution=(640, 480))
        
        # Initialize RGB annotator
        rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annotator.attach([render_product])
        
        print(f"Camera render product created: {camera_prim_path}")
        print(f"Camera resolution: 640x480")
        
        # Ensure simulation is playing
        if not my_world.is_playing():
            await my_world.play_async()
            for _ in range(20):
                my_world.step(render=True)
                await omni.kit.app.get_app().next_update_async()
        
        # Get current positions
        current_pos = my_robot.get_joint_positions()
        if current_pos is None:
            print("Error: Could not read joint positions")
            return
        
        num_joints = len(current_pos)
        print(f"Robot has {num_joints} joints")
        print(f"Starting position: {np.round(current_pos[:6], 3)}")
        if num_joints > 6:
            print(f"Starting gripper position: {current_pos[6]:.6f}")
        print(f"Executing trajectory with {len(joint_trajectory)} points...")
        print(f"Estimated duration: {len(joint_trajectory)/50.0:.2f} seconds (at 50 Hz)")
        
        articulation_controller = my_robot.get_articulation_controller()
        
        # Track timing for accurate execution
        import time
        start_time = time.time()
        
        # Execute trajectory - hold each position for 2 steps to slow down to ~25 Hz
        steps_per_point = 2  # Increase this to slow down more (3 = ~16.6 Hz, 4 = ~12.5 Hz)
        
        for point_idx, (target_joints, target_gripper) in enumerate(zip(joint_trajectory, gripper_trajectory)):
            # Create full joint position array
            target_joint_positions = current_pos.copy()
            target_joint_positions[:6] = target_joints
            
            # Set gripper position if robot has gripper joints
            if num_joints > 6:
                target_joint_positions[6] = target_gripper
            
            # Apply this position
            action = ArticulationAction(joint_positions=target_joint_positions)
            articulation_controller.apply_action(action)
            
            # Hold this position for multiple simulation steps to slow down
            for step in range(steps_per_point):
                my_world.step(render=True)
                
                # Only yield to event loop occasionally to keep UI responsive
                if (point_idx * steps_per_point + step) % 10 == 0:
                    await omni.kit.app.get_app().next_update_async()
            
            # Capture camera image after robot reaches position
            # Get RGB data from annotator
            rgb_data = rgb_annotator.get_data()
            
            if rgb_data is not None:
                # rgb_data is in shape (height, width, 4) with RGBA
                # Take only RGB channels (drop alpha)
                rgb_image = rgb_data[:, :, :3]
                
                # Ensure it's uint8
                if rgb_image.dtype != np.uint8:
                    rgb_image = (rgb_image * 255).astype(np.uint8)
                
                # Verify shape is (480, 640, 3)
                if rgb_image.shape != (480, 640, 3):
                    print(f"Warning: Unexpected image shape {rgb_image.shape}, expected (480, 640, 3)")
                
                # Save as PNG with frame number (1-indexed, 4 digits)
                frame_filename = f"frame_{point_idx+1:04d}.png"
                frame_path = os.path.join(camera_output_dir, frame_filename)
                
                # Save using PIL
                img = Image.fromarray(rgb_image, mode='RGB')
                img.save(frame_path)
                
                # Print progress every 100 frames
                if (point_idx + 1) % 100 == 0:
                    print(f"Saved frame {point_idx+1}/{len(joint_trajectory)}: {frame_filename}")
            else:
                print(f"Warning: No camera data at frame {point_idx+1}")
            
            # Print progress every 500 steps
            if point_idx % 500 == 0:
                elapsed_time = time.time() - start_time
                expected_time = point_idx / 50.0 * steps_per_point
                print(f"Progress: {point_idx}/{len(joint_trajectory)} ({100*point_idx//len(joint_trajectory)}%) - Elapsed: {elapsed_time:.2f}s (expected: {expected_time:.2f}s) - Gripper: {target_gripper:.6f}")
        
        elapsed = time.time() - start_time
        final_pos = my_robot.get_joint_positions()
        print(f"Final position: {np.round(final_pos[:6], 3)}")
        if num_joints > 6:
            print(f"Final gripper position: {final_pos[6]:.6f}")
        print(f"Trajectory execution complete!")
        print(f"Total time: {elapsed:.2f}s (at {50.0/steps_per_point:.1f} Hz)")
        print(f"Saved {len(joint_trajectory)} camera frames to {camera_output_dir}")
        
    except Exception as e:
        print(f"Error during trajectory execution: {e}")
        import traceback
        traceback.print_exc()
    finally:
        _is_running = False

# Load trajectory from file
trajectory_file = "/home/azureuser/IsaacSim/projects/episode_0_full.txt"
print(f"Loading trajectory from: {trajectory_file}")
original_joints, original_gripper = load_trajectory_from_file(trajectory_file)

if original_joints is not None and len(original_joints) > 0:
    print(f"Successfully loaded {len(original_joints)} trajectory points")
    
    # Use all original trajectory points without downsampling
    final_joints = original_joints
    final_gripper = original_gripper
    
    print(f"Using all {len(final_joints)} trajectory points (no downsampling)")
    print(f"First joint point: {final_joints[0]}")
    print(f"Last joint point: {final_joints[-1]}")
    print(f"First gripper: {final_gripper[0]:.6f}, Last gripper: {final_gripper[-1]:.6f}")
    print(f"Duration: {len(final_joints)/50.0:.2f} seconds at 50 Hz")
    
    # Execute the trajectory with camera capture
    asyncio.ensure_future(execute_trajectory_with_camera(final_joints, final_gripper, robot_name="ur5e"))
else:
    print("Failed to load trajectory. Please check the file path and format.")
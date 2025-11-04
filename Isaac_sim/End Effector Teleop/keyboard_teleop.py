import numpy as np
import asyncio
import os
from isaacsim.core.api import World
import omni.kit.app
from isaacsim.core.prims import Articulation
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from isaacsim.core.utils.prims import get_prim_at_path
from pxr import Usd
import threading
import time

# Global variables
_is_running = False
_current_key = None
_stop_input_thread = False

def input_thread_function():
    """Separate thread for keyboard input - reads keys without requiring Enter"""
    global _stop_input_thread, _current_key
    
    print("\n" + "="*60)
    print("KEYBOARD CONTROL MODE")
    print("="*60)
    print("Cartesian control for UR10e end-effector:")
    print("HOLD keys for continuous movement:")
    print("W: Move forward (positive X)")
    print("S: Move backward (negative X)")
    print("A: Move left (negative Y)")
    print("D: Move right (positive Y)")
    print("Q: Move up (positive Z)")
    print("E: Move down (negative Z)")
    print("Press 'L' to stop")
    print("="*60 + "\n")
    
    try:
        import sys
        import tty
        import termios
        
        # Save terminal settings
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            # Set terminal to cbreak mode (better than raw for this use case)
            tty.setcbreak(sys.stdin.fileno())
            
            last_key = None  # Track last key to avoid duplicate prints
            
            while not _stop_input_thread:
                # Use a simple blocking read with timeout-like behavior
                import select
                
                # Check if input is available (with 0.05s timeout)
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                
                if rlist:
                    key = sys.stdin.read(1).lower()
                    _current_key = key
                    
                    # Echo the key press for feedback only if it's a new key
                    if key in ['w', 's', 'a', 'd', 'q', 'e'] and key != last_key:
                        print(f"Key pressed: {key.upper()}", flush=True)
                        last_key = key
                    
                    if key == 'l':
                        print("\nStopping control...")
                        break
                else:
                    # No key pressed in timeout period, clear current key
                    _current_key = None
                    last_key = None  # Reset last key when no key is pressed
                    
        finally:
            # Restore terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print("\nTerminal restored.")
            
    except Exception as e:
        print(f"Input thread error: {e}")
        import traceback
        traceback.print_exc()

def find_robot_prim_path(robot_name="ur"):
    """Find the actual prim path of the robot in the stage"""
    from isaacsim.core.utils.stage import get_current_stage
    
    stage = get_current_stage()
    possible_paths = [
        f"/{robot_name}",
        f"/World/{robot_name}",
        f"/UR",
        f"/World/UR",
        f"/ur",
        f"/World/ur",
    ]
    
    for path in possible_paths:
        prim = get_prim_at_path(path)
        if prim and prim.IsValid():
            print(f"Found robot at: {path}")
            return path
    
    # If not found, search the entire stage
    print("Searching stage for UR robot...")
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        prim_name = prim.GetName().lower()
        if "ur" in prim_name:
            path = str(prim.GetPath())
            print(f"Found possible robot at: {path}")
            return path
    
    return None

class KinematicsSolver(ArticulationKinematicsSolver):
    def __init__(self, robot_articulation, end_effector_frame_name=None):
        base_path = os.path.dirname(__file__)
        self._kinematics = LulaKinematicsSolver(
            robot_description_path=os.path.join(base_path, "robot_descriptor.yaml"),
            urdf_path=os.path.join(base_path, "ur10e.urdf"),
        )
        if end_effector_frame_name is None:
            end_effector_frame_name = "ee_link_robotiq_arg2f_base_link"
        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)
        return

async def keyboard_control_loop(robot_name="ur"):
    """
    Control UR10e end-effector using keyboard input for Cartesian movement.
    """
    global _is_running, _current_key, _stop_input_thread
    
    if _is_running:
        print("Already running! Please wait...")
        return
    
    _is_running = True
    
    try:
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
                print(f"Error: Could not find UR robot in the stage. Please load it first through GUI.")
                return
            
            print(f"Registering robot from {robot_prim_path}...")
            from isaacsim.robot.manipulators.manipulators import SingleManipulator
            my_robot = SingleManipulator(
                prim_path=robot_prim_path,
                name=robot_name,
                end_effector_prim_path=f"{robot_prim_path}/ee_link/robotiq_arg2f_base_link"
            )
            my_world.scene.add(my_robot)
            await my_world.reset_async()
            print(f"Robot '{robot_name}' registered successfully")
        
        # Ensure simulation is playing
        if not my_world.is_playing():
            await my_world.play_async()
            for _ in range(20):
                my_world.step(render=True)
                await omni.kit.app.get_app().next_update_async()
        
        # Initialize IK solver
        ik_solver = KinematicsSolver(my_robot)
        articulation_controller = my_robot.get_articulation_controller()
        
        # Movement step size (meters) - smaller for continuous movement
        step_size = 0.025
        
        # Start input thread
        _stop_input_thread = False
        input_thread = threading.Thread(target=input_thread_function, daemon=True)
        input_thread.start()
        
        # Give input thread time to initialize (using time.sleep in non-async context)
        time.sleep(0.5)
        
        # Control loop
        running = True
        movement_count = 0
        last_position = None
        update_counter = 0
        
        while running:
            # Check current key being pressed
            current_key = _current_key
            
            if current_key == 'l':
                print("\nExiting keyboard control mode...")
                running = False
                break
            
            # If a valid movement key is pressed, move the robot
            if current_key in ['w', 's', 'a', 'd', 'q', 'e']:
                # Get current end-effector pose
                current_position, current_orientation = my_robot.end_effector.get_world_pose()
                
                # Modify position based on input
                if current_key == 'w':
                    current_position[0] += step_size  # Forward (positive X)
                elif current_key == 's':
                    current_position[0] -= step_size  # Backward (negative X)
                elif current_key == 'a':
                    current_position[1] -= step_size  # Left (negative Y)
                elif current_key == 'd':
                    current_position[1] += step_size  # Right (positive Y)
                elif current_key == 'q':
                    current_position[2] += step_size  # Up (positive Z)
                elif current_key == 'e':
                    current_position[2] -= step_size  # Down (negative Z)
                
                # Compute IK for new pose
                actions, succ = ik_solver.compute_inverse_kinematics(
                    target_position=current_position,
                    target_orientation=current_orientation,
                )
                
                if succ:
                    articulation_controller.apply_action(actions)
                    
                    # Print position every 50 movements to avoid spam
                    movement_count += 1
                    if movement_count % 50 == 0:
                        print(f"\nEnd-effector at: {np.round(current_position, 3)}")
                    
                    last_position = current_position.copy()
                else:
                    if last_position is not None:
                        print(f"\nIK failed - at limit. Current pos: {np.round(last_position, 3)}")
            
            # Step simulation
            my_world.step(render=True)
            
            # Yield to event loop periodically to keep streaming alive
            update_counter += 1
            if update_counter % 3 == 0:
                await omni.kit.app.get_app().next_update_async()
        
        # Stop input thread
        _stop_input_thread = True
        input_thread.join(timeout=1.0)
        print("Keyboard control stopped.")
        
    except Exception as e:
        print(f"Error during keyboard control: {e}")
        import traceback
        traceback.print_exc()
    finally:
        _is_running = False
        _stop_input_thread = True

# Start keyboard control
asyncio.ensure_future(keyboard_control_loop(robot_name="ur"))
#!/usr/bin/env python3
"""
Extract joint positions + gripper positions from HDF5 and detect gripper events.
Usage:
  python3 extract_joints_with_gripper.py <input.h5> <output.txt> [--rate HZ]

Output format:
  time joint1 joint2 joint3 joint4 joint5 joint6 gripper

Also generates <output>_gripper_events.txt with gripper open/close timestamps.
"""
import sys
import argparse
import h5py
import numpy as np


def detect_gripper_events(gripper_positions, times, threshold=0.01):
    """
    Detect when gripper opens or closes.
    Returns list of (index, time, event_type, position) tuples.
    event_type: 'open' or 'close'
    """
    events = []
    prev_pos = gripper_positions[0]
    prev_direction = None
    
    for i in range(1, len(gripper_positions)):
        curr_pos = gripper_positions[i]
        delta = curr_pos - prev_pos
        
        # Detect significant change
        if abs(delta) > threshold:
            if delta > 0:  # Opening
                direction = 'open'
            else:  # Closing
                direction = 'close'
            
            # Only record if direction changed
            if direction != prev_direction:
                events.append((i, times[i], direction, float(curr_pos)))
                prev_direction = direction
        
        prev_pos = curr_pos
    
    return events


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('h5file', help='HDF5 file path')
    parser.add_argument('outfile', help='Output text file path')
    parser.add_argument('--rate', type=float, default=50.0, help='Sampling rate in Hz (default: 50)')
    parser.add_argument('--arm-dataset', default='/action/ur5e/joints/position', help='HDF5 dataset path for arm joints')
    parser.add_argument('--gripper-dataset', default='/action/robotiq_gripper/gripper/position', help='HDF5 dataset path for gripper')
    parser.add_argument('--threshold', type=float, default=0.01, help='Gripper change threshold for event detection')
    args = parser.parse_args()

    with h5py.File(args.h5file, 'r') as f:
        arm_positions = f[args.arm_dataset][()]
        gripper_positions = f[args.gripper_dataset][()]
    
    n_points, n_joints = arm_positions.shape
    gripper_positions = gripper_positions.squeeze()  # Remove extra dimension if (N, 1)
    
    dt = 1.0 / args.rate
    times = np.arange(n_points) * dt

    # Write joint + gripper data
    with open(args.outfile, 'w') as out:
        out.write(f"# Joint + gripper positions extracted from {args.h5file}\n")
        out.write(f"# Arm dataset: {args.arm_dataset}\n")
        out.write(f"# Gripper dataset: {args.gripper_dataset}\n")
        out.write(f"# Rate: {args.rate} Hz\n")
        out.write(f"# Format: time joint1 joint2 joint3 joint4 joint5 joint6 gripper\n")
        for t, arm_pos, grip_pos in zip(times, arm_positions, gripper_positions):
            line = f"{t:.6f}"
            for p in arm_pos:
                line += f" {p:.6f}"
            line += f" {grip_pos:.6f}\n"
            out.write(line)
    
    print(f"Extracted {n_points} points ({n_joints} arm joints + gripper) to {args.outfile}")
    
    # Detect and save gripper events
    events = detect_gripper_events(gripper_positions, times, args.threshold)
    events_file = args.outfile.replace('.txt', '_gripper_events.txt')
    with open(events_file, 'w') as out:
        out.write(f"# Gripper events detected from {args.h5file}\n")
        out.write(f"# Threshold: {args.threshold}\n")
        out.write(f"# Format: index time event_type joint1 joint2 joint3 joint4 joint5 joint6 gripper_position\n")
        for idx, t, event_type, pos in events:
            arm_pos = arm_positions[idx]
            line = f"{idx} {t:.6f} {event_type}"
            for p in arm_pos:
                line += f" {p:.6f}"
            line += f" {pos:.6f}\n"
            out.write(line)
    
    print(f"Detected {len(events)} gripper events, saved to {events_file}")
    
    # Print summary
    print("\nGripper events summary:")
    for idx, t, event_type, pos in events:
        print(f"  Point {idx:4d} at {t:.3f}s: {event_type:6s} (gripper: {pos:.3f})")

if __name__ == '__main__':
    main()

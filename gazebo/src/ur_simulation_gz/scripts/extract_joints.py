#!/usr/bin/env python3
"""
Extract joint positions from HDF5 file and save as text.
Usage:
  python3 extract_joints.py <input.h5> <output.txt> [--rate HZ]

Output format (one line per timestep):
  time joint1 joint2 joint3 joint4 joint5 joint6
"""
import sys
import argparse
import h5py
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('h5file', help='HDF5 file path')
    parser.add_argument('outfile', help='Output text file path')
    parser.add_argument('--rate', type=float, default=50.0, help='Sampling rate in Hz (default: 50)')
    parser.add_argument('--dataset', default='/action/ur5e/joints/position', help='HDF5 dataset path for joint positions')
    args = parser.parse_args()

    with h5py.File(args.h5file, 'r') as f:
        positions = f[args.dataset][()]
    
    n_points, n_joints = positions.shape
    dt = 1.0 / args.rate
    times = np.arange(n_points) * dt

    with open(args.outfile, 'w') as out:
        out.write(f"# Joint positions extracted from {args.h5file}\n")
        out.write(f"# Dataset: {args.dataset}\n")
        out.write(f"# Rate: {args.rate} Hz\n")
        out.write(f"# Format: time joint1 joint2 joint3 joint4 joint5 joint6\n")
        for t, pos in zip(times, positions):
            line = f"{t:.6f}"
            for p in pos:
                line += f" {p:.6f}"
            line += "\n"
            out.write(line)
    
    print(f"Extracted {n_points} points ({n_joints} joints) to {args.outfile}")

if __name__ == '__main__':
    main()

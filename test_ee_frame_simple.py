#!/usr/bin/env python3
"""
Simple test to verify end-effector coordinate frame visualization.
This script just initializes the robot and shows the frame - no motion.
"""

import numpy as np
import time
from deoxys.franka_interface import FrankaInterface

def main():
    print("Initializing FrankaInterface in mock mode with visualization...")
    print("This will open a PyBullet GUI window.")

    robot = FrankaInterface(
        general_cfg_file="deoxys/config/local-host.yml",
        control_freq=20.0,
        mock_mode=True,
        use_visualizer=True,
        has_gripper=False
    )

    # Wait for state buffer to populate
    print("\nWaiting for robot state to initialize...")
    while robot.state_buffer_size == 0:
        time.sleep(0.1)

    print("\n" + "="*60)
    print("SUCCESS! The PyBullet visualizer should now show:")
    print("  1. The Franka Panda robot")
    print("  2. A coordinate frame at the end-effector with:")
    print("     - RED line   = X axis")
    print("     - GREEN line = Y axis")
    print("     - BLUE line  = Z axis")
    print("="*60)
    print("\nThe visualization will stay open for 10 seconds...")

    # Keep visualization open
    for i in range(10, 0, -1):
        print(f"\rClosing in {i} seconds... ", end='', flush=True)
        time.sleep(1)

    print("\n\nShutting down...")
    robot.close()
    print("Done!")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Test script to verify end-effector coordinate frame visualization in PyBullet.
"""

import numpy as np
import time
from deoxys.franka_interface import FrankaInterface

def main():
    # Initialize FrankaInterface in mock mode with visualizer enabled
    print("Initializing FrankaInterface in mock mode with visualization...")
    robot = FrankaInterface(
        general_cfg_file="deoxys/config/local-host.yml",
        control_freq=20.0,
        mock_mode=True,
        use_visualizer=True,
        has_gripper=False
    )

    # Wait for state buffer to populate
    print("Waiting for state buffer to populate...")
    while robot.state_buffer_size == 0:
        time.sleep(0.1)

    print("Robot initialized. The PyBullet visualizer should show:")
    print("  - The Franka Panda robot")
    print("  - A coordinate frame at the end-effector with:")
    print("    - RED axis: X direction")
    print("    - GREEN axis: Y direction")
    print("    - BLUE axis: Z direction")
    print("\nThe coordinate frame should update as the robot moves.")
    print("\nPress Ctrl+C to exit...")

    # Keep the visualization running
    try:
        # Move the robot in a small circle to show the frame updating
        t = 0
        while True:
            # Generate a simple circular motion in joint space
            joint_action = np.zeros(8)
            joint_action[0] = 0.1 * np.sin(t)
            joint_action[1] = 0.05 * np.cos(t)
            joint_action[-1] = 0.5  # Gripper (mid position)

            # Send control command
            robot.control(
                controller_type="JOINT_POSITION",
                action=joint_action,
                controller_cfg={
                    'controller_type': 'JOINT_POSITION',
                    'is_delta': True,
                    'traj_interpolator_cfg': {
                        'traj_interpolator_type': 'SMOOTH_JOINT_POSITION',
                        'time_fraction': 0.2
                    },
                    'state_estimator_cfg': {
                        'is_estimation': False,
                        'state_estimator_type': 'EXPONENTIAL_SMOOTHING',
                        'alpha_q': 0.9,
                        'alpha_dq': 0.9,
                        'alpha_eef': 1.0,
                        'alpha_eef_vel': 1.0
                    }
                }
            )

            t += 0.05
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nShutting down...")
        robot.close()

if __name__ == "__main__":
    main()

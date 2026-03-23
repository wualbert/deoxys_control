#!/usr/bin/env python3
"""
Demonstration of End-Effector Coordinate Frame Visualization in PyBullet.

This script shows the coordinate frame at the end-effector link (link index 8)
and demonstrates how it updates as the robot moves.

The coordinate frame is displayed as:
- RED line: X-axis
- GREEN line: Y-axis
- BLUE line: Z-axis
"""

import numpy as np
import time
import sys
from deoxys.franka_interface import FrankaInterface
from deoxys.utils.yaml_config import YamlConfig

def main():
    print("\n" + "="*60)
    print("END-EFFECTOR COORDINATE FRAME VISUALIZATION DEMO")
    print("="*60)

    print("\nInitializing FrankaInterface in mock mode with visualization...")
    print("A PyBullet GUI window will open.\n")

    # Initialize robot with visualizer
    robot = FrankaInterface(
        general_cfg_file="deoxys/config/local-host.yml",
        control_freq=10.0,
        mock_mode=True,
        use_visualizer=True,
        has_gripper=False
    )

    # Wait for initialization
    print("Waiting for robot to initialize...")
    timeout = 5.0
    start_time = time.time()
    while robot.state_buffer_size == 0:
        if time.time() - start_time > timeout:
            print("ERROR: Timeout waiting for robot state")
            return
        time.sleep(0.1)

    print("\n" + "-"*60)
    print("✓ SUCCESS! The PyBullet visualizer is now showing:")
    print("-"*60)
    print("  • The Franka Panda robot model")
    print("  • A coordinate frame at the end-effector (link 8)")
    print("    - RED line   = X-axis")
    print("    - GREEN line = Y-axis")
    print("    - BLUE line  = Z-axis")
    print("-"*60)

    # Get initial pose
    if robot.last_eef_pose is not None:
        ee_pose = robot.last_eef_pose
        print(f"\nEnd-effector position: [{ee_pose[0,3]:.3f}, {ee_pose[1,3]:.3f}, {ee_pose[2,3]:.3f}]")

    print("\nThe robot will now move in a simple pattern.")
    print("Watch how the coordinate frame follows the end-effector!")
    print("\nPress Ctrl+C to exit...\n")

    # Load controller config from YAML
    controller_cfg = YamlConfig("deoxys/config/joint-position-controller.yml").as_easydict()

    # Main control loop - simple sine wave motion
    try:
        t = 0
        while True:
            # Create simple joint motion pattern
            joint_action = np.zeros(8)

            # Small oscillation on joints 1 and 2
            joint_action[0] = 0.0  # Keep base fixed
            joint_action[1] = -0.785 + 0.2 * np.sin(t)  # Oscillate joint 2
            joint_action[2] = 0.0
            joint_action[3] = -2.356 + 0.1 * np.cos(t)  # Small oscillation joint 4
            joint_action[4] = 0.0
            joint_action[5] = 1.571
            joint_action[6] = 0.785
            joint_action[7] = 0.5  # Gripper (not used)

            # Modify config for this specific command
            controller_cfg.is_delta = False  # Use absolute positions

            # Send absolute joint position command
            robot.control(
                controller_type="JOINT_POSITION",
                action=joint_action,
                controller_cfg=controller_cfg
            )

            t += 0.05
            time.sleep(0.1)  # Control at 10Hz

    except KeyboardInterrupt:
        print("\n\nReceived interrupt signal. Shutting down...")
    except Exception as e:
        print(f"\nError during execution: {e}")
    finally:
        robot.close()
        print("Demo completed successfully!")

if __name__ == "__main__":
    main()
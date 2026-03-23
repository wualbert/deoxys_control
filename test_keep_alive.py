#!/usr/bin/env python3
"""
Test script to keep franka-interface alive by sending periodic heartbeat messages.
This helps diagnose if the issue is lack of client connection.
"""

import time
from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig

def main():
    # Load configuration
    config_path = config_root + "/charmander.yml"
    config = YamlConfig(config_path).as_easydict()

    print(f"Connecting to NUC at {config.NUC.IP}...")
    print(f"Ports: {config.NUC.SUB_PORT} (sub), {config.NUC.PUB_PORT} (pub)")

    try:
        # Initialize interface
        robot_interface = FrankaInterface(
            config_path,
            use_visualizer=False,  # Disable visualizer to avoid potential issues
            control_freq=20,
        )

        print("Connected! Keeping interface alive...")
        print("Press Ctrl+C to stop")

        # Keep the interface alive by just reading state
        while True:
            # Get current state
            state = robot_interface.get_robot_state()
            if state:
                print(f"Robot state received - Joint pos[0]: {state['q'][0]:.4f}")
            else:
                print("Warning: No state received")

            time.sleep(0.5)  # Check every 500ms

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
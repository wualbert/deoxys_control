"""
SpaceMouse teleoperation example for Franka robot.

This script demonstrates how to control a Franka robot using a SpaceMouse device.
It supports both physical robot control and mock mode with visualization.

Usage:
    # Run with physical robot (auto-fallback to mock if hardware unavailable)
    python run_deoxys_with_space_mouse.py

    # Explicitly run in mock mode with visualization
    python run_deoxys_with_space_mouse.py --mock --visualize

    # Run with physical robot only (fail if hardware unavailable)
    python run_deoxys_with_space_mouse.py --no-auto-mock

    # Custom SpaceMouse vendor/product IDs
    python run_deoxys_with_space_mouse.py --vendor-id 9583 --product-id 50734
"""
import argparse
import time

from deoxys.franka_interface import FrankaInterface
from deoxys.utils.config_utils import get_default_controller_config
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="config/charmander.yml")
    parser.add_argument("--controller-type", type=str, default="OSC_POSE")

    parser.add_argument("--vendor-id", type=int, default=9583)
    parser.add_argument("--product-id", type=int, default=50734)

    parser.add_argument("--mock", action="store_true",
                        help="Run in mock mode (no physical robot required)")
    parser.add_argument("--visualize", action="store_true",
                        help="Enable PyBullet visualization (only in mock mode)")
    parser.add_argument("--auto-mock", action="store_true", default=True,
                        help="Automatically fall back to mock mode if hardware connection fails")

    args = parser.parse_args()

    print("=" * 70)
    print("SpaceMouse Teleoperation for Franka Robot")
    print("=" * 70)

    device = SpaceMouse(vendor_id=args.vendor_id, product_id=args.product_id)
    device.start_control()

    # Try to connect to real hardware first, fall back to mock mode if it fails
    robot_interface = None
    if args.mock:
        logger.info("Mock mode explicitly requested - starting in mock mode")
        robot_interface = FrankaInterface(
            args.interface_cfg,
            use_visualizer=args.visualize,
            mock_mode=True
        )
    else:
        try:
            logger.info("Attempting to connect to physical robot...")
            robot_interface = FrankaInterface(args.interface_cfg, use_visualizer=False)
            logger.info("Successfully connected to physical robot")
        except Exception as e:
            if args.auto_mock:
                logger.warning(f"Failed to connect to physical robot: {e}")
                logger.info("Auto-mock enabled - falling back to mock mode with visualization")
                robot_interface = FrankaInterface(
                    args.interface_cfg,
                    use_visualizer=True,
                    mock_mode=True
                )
            else:
                logger.error("Failed to connect to physical robot and auto-mock is disabled")
                raise

    controller_type = args.controller_type
    controller_cfg = get_default_controller_config(controller_type=controller_type)
    # controller_cfg = YamlConfig("config/osc-pose-controller.yml").as_easydict()

    robot_interface._state_buffer = []

    for i in range(3000):
        start_time = time.time_ns()

        action, grasp = input2action(
            device=device,
            controller_type=controller_type,
        )

        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )
        end_time = time.time_ns()
        logger.debug(f"Time duration: {((end_time - start_time) / (10**9))}")

    robot_interface.control(
        controller_type=controller_type,
        action=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0] + [1.0],
        controller_cfg=controller_cfg,
        termination=True,
    )

    robot_interface.close()

    # Check if there is any state frame missing
    for state, next_state in zip(
        robot_interface._state_buffer[:-1], robot_interface._state_buffer[1:]
    ):
        if (next_state.frame - state.frame) > 1:
            print(state.frame, next_state.frame)


if __name__ == "__main__":
    main()

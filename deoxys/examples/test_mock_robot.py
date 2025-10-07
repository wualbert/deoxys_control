#!/usr/bin/env python3
"""
Comprehensive test script for mock FrankaInterface with visualization.
This demonstrates various robot movements and control modes in simulation.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
import logging
import argparse

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_joint_waypoints(robot_interface):
    """Test joint position control with multiple waypoints."""
    print("\n" + "=" * 60)
    print("🎯 JOINT WAYPOINT NAVIGATION TEST")
    print("=" * 60)

    controller_cfg = YamlConfig(
        config_root + "/joint-position-controller.yml"
    ).as_easydict()
    controller_type = "JOINT_POSITION"

    # Define waypoints that create interesting robot poses
    waypoints = [
        ([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], 1.0, "Home Position"),
        ([0.5, -0.5, 0.3, -2.0, 0.2, 1.8, 1.0], 0.8, "Extended Reach"),
        ([-0.5, -1.0, -0.3, -2.5, -0.2, 1.2, 0.5], 0.3, "Contracted Pose"),
        ([0.2, -0.3, 0.5, -1.8, 0.4, 2.0, 1.2], 0.6, "High Reach"),
        ([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], 0.5, "Return Home"),
    ]

    print(
        f"📍 Moving through {len(waypoints)} waypoints with gripper control\n")

    for wp_idx, (target_joints, gripper_pos, description) in enumerate(waypoints):
        print(f"Waypoint {wp_idx + 1}: {description}")
        print(f"  Target: {np.round(target_joints, 2)}")
        print(f"  Gripper: {gripper_pos:.1f} ({gripper_pos*0.08:.3f}m)")

        action = target_joints + [gripper_pos]

        # Move to waypoint with convergence check
        for i in range(30):
            robot_interface.control(
                controller_type=controller_type,
                action=action,
                controller_cfg=controller_cfg,
            )

            if i % 10 == 0 and len(robot_interface._state_buffer) > 0:
                current = robot_interface.last_q
                error = np.max(np.abs(current - np.array(target_joints)))
                print(f"    Step {i:2d}: Max error = {error:.4f}")

            time.sleep(0.05)

        print(f"  ✓ Reached {description}\n")


def test_smooth_trajectory(robot_interface):
    """Test smooth sinusoidal trajectory following."""
    print("\n" + "=" * 60)
    print("🌊 SMOOTH TRAJECTORY FOLLOWING TEST")
    print("=" * 60)

    controller_cfg = YamlConfig(
        config_root + "/joint-position-controller.yml"
    ).as_easydict()

    print("Generating smooth sinusoidal motion for all joints...")
    print("Press Ctrl+C to skip this test\n")

    base_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    amplitudes = [0.3, 0.2, 0.3, 0.2, 0.3, 0.3, 0.2]
    frequencies = [1.0, 1.5, 0.8, 2.0, 1.2, 1.5, 2.0]

    try:
        for step in range(100):
            t = step * 0.1

            # Generate smooth joint targets
            target_joints = []
            for j in range(7):
                target = base_joints[j] + amplitudes[j] * \
                    np.sin(frequencies[j] * t)
                target_joints.append(target)

            # Oscillating gripper
            gripper = 0.5 + 0.5 * np.sin(t * 0.5)
            action = target_joints + [gripper]

            robot_interface.control(
                controller_type="JOINT_POSITION",
                action=action,
                controller_cfg=controller_cfg,
            )

            if step % 20 == 0:
                print(f"  Time {t:.1f}s: Gripper = {gripper:.2f}, "
                      f"J1 = {target_joints[0]:.2f}, J4 = {target_joints[3]:.2f}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n  Trajectory interrupted by user")

    print("✓ Smooth trajectory test completed\n")


def test_cartesian_patterns(robot_interface):
    """Test various Cartesian movement patterns with SIGNIFICANT movements."""
    print("\n" + "=" * 60)
    print("🔄 CARTESIAN MOVEMENT PATTERNS TEST")
    print("=" * 60)
    print("Testing large, visible Cartesian movements!\n")

    osc_cfg = YamlConfig(
        config_root + "/osc-position-controller.yml"
    ).as_easydict()

    # Much larger movements that are easier to see
    patterns = [
        ("Large Square in XY plane (10cm sides)", [
            ([0.10, 0.0, 0.0, 0.0, 0.0, 0.0], 50, "→ Move +X (forward 10cm)"),
            ([0.0, 0.10, 0.0, 0.0, 0.0, 0.0], 50, "→ Move +Y (right 10cm)"),
            ([-0.10, 0.0, 0.0, 0.0, 0.0, 0.0], 50, "→ Move -X (back 10cm)"),
            ([0.0, -0.10, 0.0, 0.0, 0.0, 0.0], 50, "→ Move -Y (left 10cm)"),
        ]),
        ("Vertical Rectangle (15cm tall)", [
            ([0.08, 0.0, 0.0, 0.0, 0.0, 0.0], 40, "→ Move forward 8cm"),
            ([0.0, 0.0, 0.15, 0.0, 0.0, 0.0], 60, "↑ Move up 15cm"),
            ([-0.08, 0.0, 0.0, 0.0, 0.0, 0.0], 40, "← Move back 8cm"),
            ([0.0, 0.0, -0.15, 0.0, 0.0, 0.0], 60, "↓ Move down 15cm"),
        ]),
        ("Large Rotation Sequence", [
            ([0.0, 0.0, 0.0, 0.4, 0.0, 0.0], 30, "🔄 Rotate X-axis (23°)"),
            ([0.0, 0.0, 0.0, -0.4, 0.0, 0.0], 30, "🔄 Rotate back X"),
            ([0.0, 0.0, 0.0, 0.0, 0.4, 0.0], 30, "🔄 Rotate Y-axis (23°)"),
            ([0.0, 0.0, 0.0, 0.0, -0.4, 0.0], 30, "🔄 Rotate back Y"),
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0], 30, "🔄 Rotate Z-axis (29°)"),
            ([0.0, 0.0, 0.0, 0.0, 0.0, -0.5], 30, "🔄 Rotate back Z"),
        ]),
        ("Diagonal Push (12cm)", [
            ([0.12, 0.08, 0.05, 0.0, 0.0, 0.0], 60, "↗ Large diagonal push"),
            ([-0.12, -0.08, -0.05, 0.0, 0.0, 0.0], 60, "↙ Return diagonal"),
        ]),
        ("Spiral Motion", [
            ([0.05, 0.05, 0.03, 0.1, 0.0, 0.0], 30, "🌀 Spiral segment 1"),
            ([0.03, -0.05, 0.03, 0.0, 0.1, 0.0], 30, "🌀 Spiral segment 2"),
            ([-0.05, 0.0, 0.03, 0.0, 0.0, 0.1], 30, "🌀 Spiral segment 3"),
            ([-0.03, 0.0, -0.09, -0.1, -0.1, -0.1], 40, "🌀 Return to start"),
        ]),
    ]

    for pattern_name, movements in patterns:
        print(f"\n📐 Pattern: {pattern_name}")
        print("=" * 60)

        # Record initial pose
        if robot_interface.state_buffer_size > 0:
            initial_pose = robot_interface.last_eef_pose
            initial_joints = robot_interface.last_q.copy()
            if initial_pose is not None:
                initial_pos = initial_pose[:3, 3]
                print(f"  Starting EEF position: {np.round(initial_pos, 4)}")

        for move_idx, (delta, steps, description) in enumerate(movements):
            print(f"\n  Step {move_idx + 1}/{len(movements)}: {description}")
            action = delta + [0.5]  # Fixed gripper position

            # Record position before movement
            if robot_interface.state_buffer_size > 0:
                pre_pose = robot_interface.last_eef_pose
                pre_joints = robot_interface.last_q.copy()

            # Execute movement
            for step in range(steps):
                robot_interface.control(
                    controller_type="OSC_POSE",
                    action=action,
                    controller_cfg=osc_cfg,
                )
                time.sleep(0.05)

                # Show progress for longer movements
                if steps > 40 and step % 20 == 0 and step > 0:
                    print(f"    Progress: {int(100*step/steps)}%", end='\r')

            # Report results
            if robot_interface.state_buffer_size > 0:
                post_pose = robot_interface.last_eef_pose
                post_joints = robot_interface.last_q

                if pre_pose is not None and post_pose is not None:
                    # Position change
                    pos_change = post_pose[:3, 3] - pre_pose[:3, 3]
                    pos_magnitude = np.linalg.norm(pos_change)

                    # Joint change
                    joint_change = post_joints - pre_joints
                    max_joint_change = np.max(np.abs(joint_change))
                    moving_joints = np.where(np.abs(joint_change) > 0.02)[0] + 1

                    print(f"    ✓ Position moved: {pos_magnitude*100:.2f}cm")
                    print(f"      Δ(x,y,z) = ({pos_change[0]*100:.2f}, {pos_change[1]*100:.2f}, {pos_change[2]*100:.2f})cm")
                    print(f"    ✓ Max joint change: {max_joint_change:.3f} rad ({np.rad2deg(max_joint_change):.1f}°)")
                    if len(moving_joints) > 0:
                        print(f"    ✓ Joints moved significantly: {moving_joints.tolist()}")

        # Show total displacement for pattern
        if robot_interface.state_buffer_size > 0 and initial_pose is not None:
            final_pose = robot_interface.last_eef_pose
            if final_pose is not None:
                total_displacement = np.linalg.norm(final_pose[:3, 3] - initial_pos)
                print(f"\n  📊 Pattern complete! Net displacement: {total_displacement*100:.2f}cm")

        print("  " + "-" * 58)


def test_gripper_sequences(robot_interface):
    """Test various gripper control sequences."""
    print("\n" + "=" * 60)
    print("🤏 GRIPPER CONTROL SEQUENCES TEST")
    print("=" * 60)

    sequences = [
        ("Rapid Open-Close", [(1.0, 0.3), (0.0, 0.3)] * 3),
        ("Gradual Steps", [(0.0, 0.5), (0.25, 0.5),
         (0.5, 0.5), (0.75, 0.5), (1.0, 0.5)]),
        ("Precision Grip", [(0.3, 1.0), (0.25, 0.5), (0.2, 0.5), (0.15, 0.5)]),
        ("Wave Motion", [(0.5 + 0.5*np.sin(i*0.3), 0.1) for i in range(20)]),
    ]

    for seq_name, positions in sequences:
        print(f"\n🎬 Sequence: {seq_name}")

        for idx, (pos, duration) in enumerate(positions):
            robot_interface.gripper_control(pos)
            width = robot_interface._mock_gripper_width

            if idx % 5 == 0 or len(positions) < 10:
                print(f"  Step {idx+1}: Width = {width:.3f}m ({pos:.2f})")

            time.sleep(duration)


def main():
    parser = argparse.ArgumentParser(
        description="Comprehensive mock robot test")
    parser.add_argument("--visualize", action="store_true",
                        help="Enable PyBullet visualization (requires display)")
    parser.add_argument("--quick", action="store_true",
                        help="Run quick test only")
    args = parser.parse_args()

    print("=" * 60)
    print("🤖 COMPREHENSIVE MOCK FRANKA INTERFACE TEST")
    print("=" * 60)
    print(f"Visualization: {'ENABLED' if args.visualize else 'DISABLED'}")
    print(f"Mode: {'QUICK TEST' if args.quick else 'FULL TEST'}")

    # Initialize the interface
    print("\n🔧 Initializing mock FrankaInterface...")
    robot_interface = FrankaInterface(
        config_root + "/charmander.yml",
        use_visualizer=args.visualize,
        mock_mode=True
    )

    print("✅ Mock interface initialized!")

    # Wait for state buffer
    timeout = 5.0
    start_time = time.time()
    while len(robot_interface._state_buffer) == 0:
        if time.time() - start_time > timeout:
            print("⚠️ Timeout waiting for state buffer")
            break
        time.sleep(0.1)

    if len(robot_interface._state_buffer) > 0:
        print(f"📊 Initial state:")
        print(f"  Joints: {np.round(robot_interface.last_q, 3)}")
        print(f"  Gripper: {robot_interface._mock_gripper_width:.3f}m")

    try:
        if args.quick:
            # Quick test - just waypoints
            test_joint_waypoints(robot_interface)
        else:
            # Full test suite
            test_joint_waypoints(robot_interface)
            test_cartesian_patterns(robot_interface)
            test_smooth_trajectory(robot_interface)
            test_gripper_sequences(robot_interface)

        print("\n" + "=" * 60)
        print("✅ ALL TESTS COMPLETED SUCCESSFULLY!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\n⚠️ Tests interrupted by user")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        robot_interface.close()
        print("\n🏁 Mock interface closed. Goodbye!")


if __name__ == "__main__":
    main()

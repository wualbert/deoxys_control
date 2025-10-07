import argparse
import os
import time

import numpy as np

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.config_utils import robot_config_parse_args
from deoxys.utils.log_utils import get_deoxys_example_logger
from deoxys.utils.ik_utils import IKWrapper


logger = get_deoxys_example_logger()


def verify_pose_with_pybullet(robot_interface, target_position, test_name=""):
    """
    Verify that the robot's actual end-effector pose matches the commanded target.
    Uses PyBullet forward kinematics for accurate verification.

    Args:
        robot_interface: FrankaInterface instance
        target_position: Commanded target position [x, y, z]
        test_name: Name of the test for logging

    Returns:
        dict with verification results
    """
    logger.info("\n" + "=" * 70)
    logger.info(f"🔍 POSE VERIFICATION: {test_name}")
    logger.info("=" * 70)

    # Get current joint configuration
    final_joints = robot_interface.last_q
    logger.info(f"Final joint configuration: {np.round(final_joints, 4)}")

    # Get EEF pose from robot interface (computed via PyBullet FK in mock mode)
    final_eef_pose = robot_interface.last_eef_pose

    if final_eef_pose is None:
        logger.error("❌ No EEF pose available from robot interface!")
        return None

    # Extract position from 4x4 transformation matrix
    actual_position = final_eef_pose[:3, 3]
    actual_orientation = final_eef_pose[:3, :3]

    logger.info(f"\n📍 Target Position (Commanded from IK):")
    logger.info(f"   X: {target_position[0]:.6f} m")
    logger.info(f"   Y: {target_position[1]:.6f} m")
    logger.info(f"   Z: {target_position[2]:.6f} m")

    logger.info(
        f"\n📍 Actual Position (PyBullet FK, link {robot_interface._ee_link_index}):")
    logger.info(f"   X: {actual_position[0]:.6f} m")
    logger.info(f"   Y: {actual_position[1]:.6f} m")
    logger.info(f"   Z: {actual_position[2]:.6f} m")

    # Compute position error
    position_error = target_position - actual_position
    position_error_magnitude = np.linalg.norm(position_error)

    logger.info(f"\n📏 Position Error:")
    logger.info(f"   ΔX: {position_error[0]*1000:+.2f} mm")
    logger.info(f"   ΔY: {position_error[1]*1000:+.2f} mm")
    logger.info(f"   ΔZ: {position_error[2]*1000:+.2f} mm")
    logger.info(f"   Total magnitude: {position_error_magnitude*1000:.2f} mm")

    # Determine if pose matches within tolerance
    tolerance_mm = 5.0  # 5mm tolerance
    tolerance_m = tolerance_mm / 1000.0

    if position_error_magnitude < tolerance_m:
        logger.info(
            f"\n✅ POSE MATCH! Error ({position_error_magnitude*1000:.2f}mm) < Tolerance ({tolerance_mm}mm)")
        match_status = "PASS"
    else:
        logger.warning(
            f"\n⚠️  POSE MISMATCH! Error ({position_error_magnitude*1000:.2f}mm) > Tolerance ({tolerance_mm}mm)")
        match_status = "FAIL"

    # Log orientation for completeness
    logger.info(f"\n🔄 Orientation Matrix (Rotation part of EEF pose):")
    logger.info(f"   {actual_orientation[0]}")
    logger.info(f"   {actual_orientation[1]}")
    logger.info(f"   {actual_orientation[2]}")

    # Additional verification: If in mock mode, we can directly access PyBullet
    if robot_interface.mock_mode and hasattr(robot_interface, '_pybullet_uid'):
        try:
            import pybullet as p

            logger.info(f"\n🔬 Direct PyBullet Verification:")

            # Update PyBullet state
            for i in range(7):
                p.resetJointState(
                    robot_interface._pybullet_robot_uid, i,
                    final_joints[i],
                    physicsClientId=robot_interface._pybullet_uid
                )

            # Get EEF state directly from PyBullet
            link_state = p.getLinkState(
                robot_interface._pybullet_robot_uid,
                robot_interface._ee_link_index,
                computeForwardKinematics=True,
                physicsClientId=robot_interface._pybullet_uid
            )

            pybullet_pos = np.array(link_state[4])
            pybullet_orn = np.array(link_state[5])

            logger.info(f"   PyBullet EEF position: {pybullet_pos}")
            logger.info(f"   PyBullet EEF orientation (quat): {pybullet_orn}")

            # Verify consistency between robot interface and direct PyBullet
            consistency_error = np.linalg.norm(pybullet_pos - actual_position)
            logger.info(
                f"   Consistency check: {consistency_error*1000:.4f} mm")

            if consistency_error < 0.001:  # 1mm
                logger.info(f"   ✅ PyBullet FK is consistent!")
            else:
                logger.warning(
                    f"   ⚠️  Inconsistency detected between interface and direct PyBullet!")

        except Exception as e:
            logger.warning(f"   Direct PyBullet verification failed: {e}")

    logger.info("=" * 70)

    return {
        'target_position': target_position,
        'actual_position': actual_position,
        'position_error': position_error,
        'error_magnitude_mm': position_error_magnitude * 1000,
        'match_status': match_status,
        'tolerance_mm': tolerance_mm,
        'final_joints': final_joints,
        'eef_pose': final_eef_pose
    }


def execute_ik_result(robot_interface, controller_type, controller_cfg, joint_traj, auto_execute=False):
    if auto_execute:
        logger.info("Auto-executing trajectory in mock mode...")
        execute = True
    else:
        valid_input = False
        while not valid_input:
            try:
                execute = input(
                    f"Execute trajectory? (enter 0 - No or 1 - Yes): ")
                execute = bool(int(execute))
                valid_input = True
            except ValueError:
                print("Please input 1 or 0!")
                continue

    if execute:
        logger.info(
            f"Executing IK trajectory with {len(joint_traj)} waypoints...")
        for i, joint in enumerate(joint_traj):
            # This example assumes the gripper is open
            action = joint.tolist() + [-1.0]
            robot_interface.control(
                controller_type=controller_type,
                action=action,
                controller_cfg=controller_cfg,
            )
            if (i + 1) % 20 == 0:
                logger.info(f"  Progress: {i+1}/{len(joint_traj)} waypoints")
        logger.info("✓ Trajectory execution complete!")
    else:
        logger.info("Trajectory execution skipped")

    return bool(execute)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--motion-file", type=str)
    parser.add_argument("--mock", action="store_true",
                        help="Run in mock mode without real hardware")
    parser.add_argument("--visualize", action="store_true",
                        help="Enable PyBullet visualization in mock mode")
    robot_config_parse_args(parser)
    return parser.parse_args()


def main():

    args = parse_args()

    if args.mock:
        logger.info(
            "Running in MOCK MODE - no real hardware connection required")

    # Franka Interface
    robot_interface = FrankaInterface(
        os.path.join(config_root, args.interface_cfg),
        use_visualizer=args.visualize if args.mock else False,
        mock_mode=args.mock
    )

    controller_cfg = YamlConfig(os.path.join(
        config_root, "joint-impedance-controller.yml")).as_easydict()
    controller_type = "JOINT_IMPEDANCE"

    # Wait for robot state
    timeout = 5.0 if args.mock else 10.0
    start_time = time.time()
    while robot_interface.state_buffer_size == 0:
        if time.time() - start_time > timeout:
            logger.error("Timeout waiting for robot state!")
            robot_interface.close()
            return
        logger.warning("Waiting for robot state...")
        time.sleep(0.5)

    last_q = np.array(robot_interface.last_q)
    last_eef_rot, last_eef_pos = robot_interface.last_eef_rot_and_pos

    logger.info(f"Initial joint configuration: {np.round(last_q, 3)}")
    # last_eef_pos is a 3x1 array, extract just the position vector
    initial_pos = last_eef_pos.flatten(
    )[:3] if last_eef_pos is not None else None
    logger.info(
        f"Initial EEF position: {np.round(initial_pos, 4) if initial_pos is not None else 'N/A'}")

    ik_wrapper = IKWrapper()

    # Use flange frame for IK to match PyBullet's link 8 (flange frame)
    # This eliminates the 97mm gripper offset
    logger.info("\n✅ Using flange frame for IK (has_gripper=False)")
    logger.info(
        "   MuJoCo IK will target 'flange_site' to match PyBullet link 8\n")

    # Test 1: IK to absolute position
    logger.info("\n" + "=" * 60)
    logger.info("TEST 1: IK to absolute position")
    logger.info("=" * 60)
    target_world_position = np.array([0.4, 0.2, 0.25])
    logger.info(f"Target position: {target_world_position}")

    # inverse kinematics will compute the trajectory based on the current joint configuration
    # Use has_gripper=False to target the flange frame instead of gripper tip
    joint_traj, debug_info = ik_wrapper.ik_trajectory_to_target_position(
        target_world_position, last_q.tolist(), has_gripper=False, num_points=100
    )

    logger.info("Visualizing IK results in MuJoCo simulation...")
    joint_traj = ik_wrapper.interpolate_dense_traj(joint_traj)
    ik_wrapper.simulate_joint_sequence(joint_traj)

    executed = execute_ik_result(
        robot_interface, controller_type, controller_cfg, joint_traj, auto_execute=args.mock)

    if executed:
        # Wait for robot to settle
        time.sleep(0.5)

        # Verify pose using PyBullet forward kinematics
        verification_result_1 = verify_pose_with_pybullet(
            robot_interface,
            target_world_position,
            test_name="TEST 1 - Absolute Position"
        )

        # Update last_q for next trajectory
        last_q = np.array(robot_interface.last_q)
        logger.info(f"Current joint configuration: {np.round(last_q, 3)}")

    # Test 2: IK for delta position
    logger.info("\n" + "=" * 60)
    logger.info("TEST 2: IK for delta position movement")
    logger.info("=" * 60)
    delta_position = np.array([0.15, 0., 0.0])
    logger.info(f"Delta position: {delta_position}")

    # Compute expected target position (current + delta)
    current_eef_rot, current_eef_pos = robot_interface.last_eef_rot_and_pos
    # current_eef_pos is 3x1, flatten and convert to 1D array
    current_pos_1d = current_eef_pos.flatten()[:3] if len(
        current_eef_pos.flatten()) >= 3 else current_eef_pos.flatten()
    expected_target_position = current_pos_1d + delta_position
    logger.info(f"Current EEF position: {np.round(current_pos_1d, 4)}")
    logger.info(
        f"Expected target position: {np.round(expected_target_position, 4)}")

    # inverse kinematics will compute the trajectory based on the current joint configuration
    # Use has_gripper=False to target the flange frame instead of gripper tip
    joint_traj, debug_info = ik_wrapper.ik_trajectory_delta_position(
        delta_position, last_q.tolist(), has_gripper=False, num_points=100
    )

    logger.info("Visualizing IK results in MuJoCo simulation...")
    joint_traj = ik_wrapper.interpolate_dense_traj(joint_traj)
    ik_wrapper.simulate_joint_sequence(joint_traj)

    executed = execute_ik_result(
        robot_interface, controller_type, controller_cfg, joint_traj, auto_execute=args.mock)

    if executed:
        # Wait for robot to settle
        time.sleep(0.5)

        # Verify pose using PyBullet forward kinematics
        verification_result_2 = verify_pose_with_pybullet(
            robot_interface,
            expected_target_position,
            test_name="TEST 2 - Delta Position"
        )

    # Final Summary
    logger.info("\n" + "=" * 70)
    logger.info("📊 FINAL SUMMARY - IK POSE VERIFICATION")
    logger.info("=" * 70)

    if 'verification_result_1' in locals() and verification_result_1:
        logger.info(f"\nTest 1 (Absolute Position):")
        logger.info(f"  Target: {verification_result_1['target_position']}")
        logger.info(
            f"  Actual: {np.round(verification_result_1['actual_position'], 6)}")
        logger.info(
            f"  Error:  {verification_result_1['error_magnitude_mm']:.2f} mm")
        logger.info(f"  Status: {verification_result_1['match_status']}")

    if 'verification_result_2' in locals() and verification_result_2:
        logger.info(f"\nTest 2 (Delta Position):")
        logger.info(f"  Target: {verification_result_2['target_position']}")
        logger.info(
            f"  Actual: {np.round(verification_result_2['actual_position'], 6)}")
        logger.info(
            f"  Error:  {verification_result_2['error_magnitude_mm']:.2f} mm")
        logger.info(f"  Status: {verification_result_2['match_status']}")

    # Overall assessment
    all_passed = True
    if 'verification_result_1' in locals() and verification_result_1:
        all_passed = all_passed and (
            verification_result_1['match_status'] == 'PASS')
    if 'verification_result_2' in locals() and verification_result_2:
        all_passed = all_passed and (
            verification_result_2['match_status'] == 'PASS')

    if all_passed:
        logger.info("\n✅ ALL TESTS PASSED! IK and FK are consistent!")
        logger.info(
            "   The commanded positions match the actual EEF positions (within 5mm).")
        logger.info(
            "   Both MuJoCo IK (flange_site) and PyBullet FK (link 8) use the flange frame.")
    else:
        logger.warning("\n⚠️  Some tests failed (error > 5mm).")
        logger.warning(
            "   This indicates a problem with IK solver or FK computation.")
        logger.warning(
            "   Both should be using the flange frame - investigate the discrepancy!")

    logger.info("=" * 70)

    robot_interface.close()
    logger.info("\n✓ IK example completed successfully!")


if __name__ == "__main__":
    main()

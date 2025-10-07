import logging
import threading
import time
from multiprocessing import Process
from typing import Tuple, Type, Union

import numpy as np
import zmq

import deoxys.proto.franka_interface.franka_controller_pb2 as franka_controller_pb2
import deoxys.proto.franka_interface.franka_robot_state_pb2 as franka_robot_state_pb2
from deoxys.franka_interface.visualizer import visualizer_factory
from deoxys.utils import transform_utils
from deoxys.utils.config_utils import verify_controller_config
from deoxys.utils.yaml_config import YamlConfig

logger = logging.getLogger(__name__)


def action_to_osc_pose_goal(action, is_delta=True) -> franka_controller_pb2.Goal:
    goal = franka_controller_pb2.Goal()
    goal.is_delta = is_delta
    goal.x = action[0]
    goal.y = action[1]
    goal.z = action[2]
    goal.ax = action[3]
    goal.ay = action[4]
    goal.az = action[5]
    return goal


def action_to_cartesian_velocity(action, is_delta=True) -> franka_controller_pb2.Goal:
    goal = franka_controller_pb2.Goal()
    goal.is_delta = is_delta
    goal.x = action[0]
    goal.y = action[1]
    goal.z = action[2]
    goal.ax = action[3]
    goal.ay = action[4]
    goal.az = action[5]
    return goal


def action_to_joint_pos_goal(action, is_delta=False) -> franka_controller_pb2.JointGoal:
    goal = franka_controller_pb2.JointGoal()
    goal.is_delta = is_delta
    goal.q1 = action[0]
    goal.q2 = action[1]
    goal.q3 = action[2]
    goal.q4 = action[3]
    goal.q5 = action[4]
    goal.q6 = action[5]
    goal.q7 = action[6]
    return goal


TRAJ_INTERPOLATOR_MAPPING = {
    "SMOOTH_JOINT_POSITION": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.SMOOTH_JOINT_POSITION,
    "LINEAR_POSE": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.LINEAR_POSE,
    "LINEAR_POSITION": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.LINEAR_POSITION,
    "LINEAR_JOINT_POSITION": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.LINEAR_JOINT_POSITION,
    "MIN_JERK_POSE": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.MIN_JERK_POSE,
    "MIN_JERK_JOINT_POSITION": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.MIN_JERK_JOINT_POSITION,
    "COSINE_CARTESIAN_VELOCITY": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.COSINE_CARTESIAN_VELOCITY,
    "LINEAR_CARTESIAN_VELOCITY": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.LINEAR_CARTESIAN_VELOCITY,
}


class FrankaInterface:
    """
    This is the Python Interface for communicating with franka interface on NUC.
    Args:
        general_cfg_file (str, optional): _description_. Defaults to "config/local-host.yml".
        control_freq (float, optional): _description_. Defaults to 20.0.
        state_freq (float, optional): _description_. Defaults to 100.0.
        control_timeout (float, optional): _description_. Defaults to 1.0.
        has_gripper (bool, optional): _description_. Defaults to True.
        use_visualizer (bool, optional): _description_. Defaults to False.
        mock_mode (bool, optional): If True, runs in mock mode without real hardware. Defaults to False.
    """

    def _init_pybullet_robot(self):
        """Initialize PyBullet robot for accurate kinematics in mock mode."""
        import pybullet as p
        import pathlib

        FILE_PATH = pathlib.Path(__file__).parent.absolute()

        # If visualizer is being used, it will handle PyBullet GUI connection
        # We only need a separate DIRECT connection for kinematics calculations
        if self.use_visualizer:
            # The visualizer will create its own GUI connection
            # We create a separate DIRECT connection for kinematics
            self._pybullet_uid = p.connect(p.DIRECT)
        else:
            # No visualizer, use DIRECT mode for kinematics only
            self._pybullet_uid = p.connect(p.DIRECT)

        # Load robot URDF for kinematics calculations
        self._pybullet_robot_uid = p.loadURDF(
            fileName=str(FILE_PATH) + "/robot_models/panda/panda.urdf",
            basePosition=[0.0, 0.0, 0.0],
            baseOrientation=[0.0, 0.0, 0.0, 1.0],
            useFixedBase=True,
            physicsClientId=self._pybullet_uid,
            flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
        )

        # End-effector link index (Panda hand - link 8 is the hand, but we need the EE frame)
        # The EE frame is typically at link index 8 for Panda
        self._ee_link_index = 8

        # Set initial joint positions
        for i in range(7):
            p.resetJointState(
                self._pybullet_robot_uid, i,
                self._mock_joint_positions[i],
                physicsClientId=self._pybullet_uid
            )

        logger.info("PyBullet robot initialized for kinematics calculations in mock mode")

    def __init__(
        self,
        general_cfg_file: str = "config/local-host.yml",
        control_freq: float = 20.0,
        state_freq: float = 100.0,
        control_timeout: float = 1.0,
        has_gripper: bool = True,
        use_visualizer: bool = False,
        automatic_gripper_reset: bool = True,
        mock_mode: bool = False,
    ):
        self.mock_mode = mock_mode
        self.use_visualizer = use_visualizer

        # Initialize mock state variables for simulation
        if self.mock_mode:
            logger.info("Running in MOCK MODE - no real hardware connection")
            # Default joint positions for Franka robot (approximate home position)
            self._mock_joint_positions = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
            self._mock_joint_velocities = np.zeros(7)
            self._mock_desired_joint_positions = self._mock_joint_positions.copy()
            self._mock_gripper_width = 0.08  # Max open position

            # Initialize PyBullet for accurate kinematics
            self._pybullet_uid = None
            self._pybullet_robot_uid = None
            self._ee_link_index = 8  # Panda EE link index
            try:
                self._init_pybullet_robot()
            except Exception as e:
                logger.warning(f"Failed to initialize PyBullet for mock mode: {e}")
                logger.warning("Falling back to simple kinematics approximation")

            self._mock_eef_pose = self._compute_mock_eef_pose()

        general_cfg = YamlConfig(general_cfg_file).as_easydict()
        self._name = general_cfg.PC.NAME
        self._ip = general_cfg.NUC.IP if not self.mock_mode else "127.0.0.1"
        self._pub_port = general_cfg.NUC.SUB_PORT
        self._sub_port = general_cfg.NUC.PUB_PORT

        self._gripper_pub_port = general_cfg.NUC.GRIPPER_SUB_PORT
        self._gripper_sub_port = general_cfg.NUC.GRIPPER_PUB_PORT

        if not self.mock_mode:
            # Real hardware mode - set up ZMQ connections
            self._context = zmq.Context()
            self._publisher = self._context.socket(zmq.PUB)
            self._subscriber = self._context.socket(zmq.SUB)

            self._gripper_publisher = self._context.socket(zmq.PUB)
            self._gripper_subscriber = self._context.socket(zmq.SUB)

            # publisher
            self._publisher.bind(f"tcp://*:{self._pub_port}")
            self._gripper_publisher.bind(f"tcp://*:{self._gripper_pub_port}")

            # subscriber
            self._subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
            self._subscriber.connect(f"tcp://{self._ip}:{self._sub_port}")

            self._gripper_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
            self._gripper_subscriber.connect(
                f"tcp://{self._ip}:{self._gripper_sub_port}")
        else:
            # Mock mode - no real ZMQ connections needed
            self._context = None
            self._publisher = None
            self._subscriber = None
            self._gripper_publisher = None
            self._gripper_subscriber = None

        self._state_buffer = []
        self._state_buffer_idx = 0

        self._gripper_state_buffer = []
        self._gripper_buffer_idx = 0

        # control frequency
        self._control_freq = control_freq
        self._control_interval = 1.0 / self._control_freq

        # state frequency
        self._state_freq = state_freq

        # control timeout (s)
        self._control_timeout = control_timeout

        self.counter = 0
        self.termination = False

        # Start state subscription threads
        if self.mock_mode:
            self._state_sub_thread = threading.Thread(target=self._mock_state_publisher)
            self._gripper_sub_thread = threading.Thread(target=self._mock_gripper_state_publisher)
        else:
            self._state_sub_thread = threading.Thread(target=self.get_state)
            self._gripper_sub_thread = threading.Thread(target=self.get_gripper_state)

        self._state_sub_thread.daemon = True
        self._state_sub_thread.start()

        self._gripper_sub_thread.daemon = True
        self._gripper_sub_thread.start()

        self.last_time = None

        self.has_gripper = has_gripper

        self.use_visualizer = use_visualizer
        self.visualizer = None
        if self.use_visualizer:
            self.visualizer = visualizer_factory(backend="pybullet")

        self._last_controller_type = "Dummy"

        self.last_gripper_dim = -1
        self.last_gripper_action = 0

        self.last_gripper_command_counter = 0
        self._history_actions = []

        # automatically reset gripper by default
        self.automatic_gripper_reset = automatic_gripper_reset

    def get_state(self, no_block: bool = False):
        """_summary_

        Args:
            no_block (bool, optional): Decide if zmq receives messages synchronously or asynchronously. Defaults to False.
        """
        # This should not be called directly. Access the states through robot_interface.last_q and robot_interface.last_q_d
        # This is broken
        if no_block:
            recv_kwargs = {"flags": zmq.NOBLOCK}
        else:
            recv_kwargs = {}
        while True:
            try:
                franka_robot_state = franka_robot_state_pb2.FrankaRobotStateMessage()
                # message = self._subscriber.recv(flags=zmq.NOBLOCK)
                message = self._subscriber.recv(**recv_kwargs)
                franka_robot_state.ParseFromString(message)
                self._state_buffer.append(franka_robot_state)
            except:
                pass

    def get_gripper_state(self):
        while True:
            try:
                franka_gripper_state = (
                    franka_robot_state_pb2.FrankaGripperStateMessage()
                )
                message = self._gripper_subscriber.recv()
                franka_gripper_state.ParseFromString(message)
                self._gripper_state_buffer.append(franka_gripper_state)
            except:
                pass

    def _compute_mock_eef_pose(self) -> np.ndarray:
        """Compute end-effector pose using PyBullet forward kinematics.
        Returns a 4x4 transformation matrix in column-major format (for protobuf compatibility).
        """
        if hasattr(self, '_pybullet_uid') and self._pybullet_uid is not None:
            import pybullet as p
            try:
                # Update joint positions in PyBullet
                for i in range(7):
                    p.resetJointState(
                        self._pybullet_robot_uid, i,
                        self._mock_joint_positions[i],
                        physicsClientId=self._pybullet_uid
                    )

                # Force forward kinematics computation
                p.stepSimulation(physicsClientId=self._pybullet_uid)

                # Get end-effector state
                link_state = p.getLinkState(
                    self._pybullet_robot_uid,
                    self._ee_link_index,
                    computeForwardKinematics=True,
                    physicsClientId=self._pybullet_uid
                )

                pos = link_state[4]  # World position of link frame origin
                orn = link_state[5]  # World orientation as quaternion (x,y,z,w)

                # Convert to 4x4 transformation matrix
                T = np.eye(4)
                T[:3, 3] = pos
                T[:3, :3] = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)

                # Return in column-major format (transpose) for protobuf
                return T.T.flatten()

            except Exception as e:
                logger.warning(f"PyBullet forward kinematics failed: {e}")
                # Fall back to approximation based on joint positions

        # Improved approximation fallback using simple FK
        # This is a rough approximation for Franka robot
        j1, j2, j3, j4, j5, j6, j7 = self._mock_joint_positions

        # Approximate end-effector position based on joint angles
        # These are rough estimates based on Franka's DH parameters
        x = 0.3 * np.cos(j1) + 0.2 * np.cos(j1 + j2) + 0.15 * np.cos(j1 + j2 + j3)
        y = 0.3 * np.sin(j1) + 0.2 * np.sin(j1 + j2) + 0.15 * np.sin(j1 + j2 + j3)
        z = 0.333 + 0.316 * np.sin(j2) + 0.0825 * np.sin(j2 + j3) + 0.384 * np.sin(j2 + j3 + j4)

        T = np.eye(4)
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z

        # Simple orientation based on wrist joints
        # This is a very rough approximation
        T[:3, :3] = self._euler_to_rotation_matrix(j5, j6, j7)

        # Return in column-major format (transpose)
        return T.T.flatten()

    def _euler_to_rotation_matrix(self, roll, pitch, yaw):
        """Convert Euler angles to rotation matrix."""
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)

        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        return R

    def _rotation_matrix_to_euler(self, R):
        """Convert rotation matrix to Euler angles (roll, pitch, yaw)."""
        # Check for gimbal lock
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])  # roll
            y = np.arctan2(-R[2, 0], sy)      # pitch
            z = np.arctan2(R[1, 0], R[0, 0])  # yaw
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        return [x, y, z]

    def _mock_state_publisher(self):
        """Publish mock robot states at the specified frequency."""
        interval = 1.0 / self._state_freq
        while not self.termination:
            try:
                # Create mock robot state message
                robot_state = franka_robot_state_pb2.FrankaRobotStateMessage()

                # Simulate smooth joint movement towards desired positions
                alpha = 0.1  # Smoothing factor
                self._mock_joint_positions = (
                    alpha * self._mock_desired_joint_positions +
                    (1 - alpha) * self._mock_joint_positions
                )

                # Set joint positions
                robot_state.q[:] = self._mock_joint_positions.tolist()
                robot_state.q_d[:] = self._mock_desired_joint_positions.tolist()
                robot_state.dq[:] = self._mock_joint_velocities.tolist()

                # Set end-effector pose (4x4 matrix in column-major format)
                robot_state.O_T_EE[:] = self._compute_mock_eef_pose().tolist()

                # Add to buffer
                self._state_buffer.append(robot_state)

                # Limit buffer size to prevent memory issues
                if len(self._state_buffer) > 1000:
                    self._state_buffer = self._state_buffer[-500:]

                time.sleep(interval)
            except Exception as e:
                logger.error(f"Error in mock state publisher: {e}")
                pass

    def _mock_gripper_state_publisher(self):
        """Publish mock gripper states at the specified frequency."""
        interval = 1.0 / self._state_freq
        while not self.termination:
            try:
                # Create mock gripper state message
                gripper_state = franka_robot_state_pb2.FrankaGripperStateMessage()

                # Set gripper width
                gripper_state.width = self._mock_gripper_width

                # Add to buffer
                self._gripper_state_buffer.append(gripper_state)

                # Limit buffer size
                if len(self._gripper_state_buffer) > 1000:
                    self._gripper_state_buffer = self._gripper_state_buffer[-500:]

                time.sleep(interval)
            except Exception as e:
                logger.error(f"Error in mock gripper state publisher: {e}")
                pass

    def _update_mock_state_simple_cartesian(self, action, controller_cfg):
        """Simple Cartesian to joint mapping when PyBullet is not available."""
        if controller_cfg.is_delta:
            cart_delta = action[:6]
            # Simple approximation of joint contributions
            x_delta, y_delta, z_delta = cart_delta[:3]
            rx_delta, ry_delta, rz_delta = cart_delta[3:6]

            joint_deltas = np.zeros(7)
            joint_deltas[0] = -y_delta * 2.0 + rz_delta * 0.5
            joint_deltas[1] = -z_delta * 1.5 - x_delta * 0.5 + ry_delta * 0.3
            joint_deltas[2] = y_delta * 1.0 - rz_delta * 0.3
            joint_deltas[3] = -z_delta * 1.2 - x_delta * 0.8 - ry_delta * 0.2
            joint_deltas[4] = rx_delta * 0.8 + y_delta * 0.5
            joint_deltas[5] = x_delta * 1.0 + z_delta * 0.5 + ry_delta * 0.5
            joint_deltas[6] = rz_delta * 1.0 + rx_delta * 0.3

            scale_factor = 2.0
            joint_deltas *= scale_factor
            self._mock_desired_joint_positions += joint_deltas

    def _update_mock_state_from_action(self, controller_type: str, action: np.ndarray, controller_cfg: dict):
        """Update mock robot state based on control action."""
        if controller_type in ["JOINT_POSITION", "JOINT_IMPEDANCE"]:
            # Both JOINT_POSITION and JOINT_IMPEDANCE update joint positions similarly
            # JOINT_IMPEDANCE just adds compliance but in mock mode we treat them the same
            if controller_cfg.is_delta:
                self._mock_desired_joint_positions += action[:7]
            else:
                self._mock_desired_joint_positions = action[:7].copy()

            # Clip to joint limits (approximate Franka limits)
            joint_limits = [
                (-2.8973, 2.8973),  # Joint 1
                (-1.7628, 1.7628),  # Joint 2
                (-2.8973, 2.8973),  # Joint 3
                (-3.0718, -0.0698), # Joint 4
                (-2.8973, 2.8973),  # Joint 5
                (-0.0175, 3.7525),  # Joint 6
                (-2.8973, 2.8973),  # Joint 7
            ]
            for i, (low, high) in enumerate(joint_limits):
                self._mock_desired_joint_positions[i] = np.clip(
                    self._mock_desired_joint_positions[i], low, high
                )

        elif controller_type in ["OSC_POSE", "OSC_POSITION", "OSC_YAW", "CARTESIAN_VELOCITY"]:
            # For Cartesian control, use PyBullet inverse kinematics and Jacobian
            if hasattr(self, '_pybullet_uid') and self._pybullet_uid is not None:
                import pybullet as p
                try:
                    # Update PyBullet state to current joint positions
                    for i in range(7):
                        p.resetJointState(
                            self._pybullet_robot_uid, i,
                            self._mock_joint_positions[i],
                            physicsClientId=self._pybullet_uid
                        )

                    # Get current end-effector pose
                    link_state = p.getLinkState(
                        self._pybullet_robot_uid,
                        self._ee_link_index,
                        computeForwardKinematics=True,
                        physicsClientId=self._pybullet_uid
                    )
                    current_pos = np.array(link_state[4])  # Current position
                    current_orn = np.array(link_state[5])  # Current orientation (quaternion)

                    if controller_cfg.is_delta:
                        # Apply delta action to current pose
                        cart_delta = action[:6].copy()

                        # Handle different controller types
                        if controller_type == "CARTESIAN_VELOCITY":
                            # For velocity control, scale by control interval
                            cart_delta *= self._control_interval

                        # Apply position delta
                        target_pos = current_pos + cart_delta[:3]

                        # Apply orientation delta based on controller type
                        if controller_type in ["OSC_POSE", "OSC_YAW"]:
                            # Convert current quaternion to rotation matrix
                            current_rot = np.array(p.getMatrixFromQuaternion(current_orn)).reshape(3, 3)

                            # Apply rotation delta
                            if controller_type == "OSC_YAW":
                                # Only apply yaw rotation
                                delta_rot = self._euler_to_rotation_matrix(0, 0, cart_delta[5])
                            else:
                                # Full orientation control
                                delta_rot = self._euler_to_rotation_matrix(cart_delta[3], cart_delta[4], cart_delta[5])

                            target_rot = current_rot @ delta_rot
                            # Convert rotation matrix to quaternion using PyBullet's Euler conversion
                            target_orn = p.getQuaternionFromEuler(self._rotation_matrix_to_euler(target_rot))
                        elif controller_type == "OSC_POSITION":
                            # Position only control, keep current orientation
                            target_orn = current_orn
                        else:  # CARTESIAN_VELOCITY
                            # Apply angular velocity
                            angular_vel = cart_delta[3:6]
                            # Simple integration for orientation
                            delta_rot = self._euler_to_rotation_matrix(angular_vel[0], angular_vel[1], angular_vel[2])
                            current_rot = np.array(p.getMatrixFromQuaternion(current_orn)).reshape(3, 3)
                            target_rot = current_rot @ delta_rot
                            # Convert rotation matrix to quaternion using PyBullet's Euler conversion
                            target_orn = p.getQuaternionFromEuler(self._rotation_matrix_to_euler(target_rot))
                    else:
                        # Absolute target (not commonly used in these controllers)
                        target_pos = action[:3]
                        target_orn = p.getQuaternionFromEuler(action[3:6])

                    # Use PyBullet inverse kinematics to find joint positions
                    # Get total number of joints for damping array
                    num_joints = p.getNumJoints(self._pybullet_robot_uid, physicsClientId=self._pybullet_uid)

                    joint_poses = p.calculateInverseKinematics(
                        self._pybullet_robot_uid,
                        self._ee_link_index,
                        target_pos,
                        target_orn,
                        lowerLimits=[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
                        upperLimits=[2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
                        jointRanges=[5.7946, 3.5256, 5.7946, 3.002, 5.7946, 3.77, 5.7946],
                        restPoses=self._mock_joint_positions[:7].tolist(),
                        jointDamping=[0.1] * num_joints,  # Damping for all joints in URDF
                        solver=p.IK_DLS,  # Damped Least Squares solver
                        maxNumIterations=100,
                        residualThreshold=1e-5,
                        physicsClientId=self._pybullet_uid
                    )

                    # Update desired joint positions
                    self._mock_desired_joint_positions = np.array(joint_poses[:7])

                    # Alternatively, for smoother control, we can also use Jacobian-based approach
                    # This is especially useful for velocity control
                    if controller_type == "CARTESIAN_VELOCITY":
                        # Calculate Jacobian for velocity mapping
                        jac_linear, jac_angular = p.calculateJacobian(
                            self._pybullet_robot_uid,
                            self._ee_link_index,
                            [0.0, 0.0, 0.0],  # Local position on end-effector
                            self._mock_joint_positions[:7].tolist(),
                            [0.0] * 7,  # Joint velocities (not used for Jacobian calculation)
                            [0.0] * 7,  # Joint accelerations (not used)
                            physicsClientId=self._pybullet_uid
                        )

                        # Stack linear and angular Jacobian (6x7 matrix)
                        J = np.vstack([np.array(jac_linear)[:, :7], np.array(jac_angular)[:, :7]])

                        # Compute pseudo-inverse with damping
                        lambda_damping = 0.01
                        J_pseudo = J.T @ np.linalg.inv(J @ J.T + lambda_damping * np.eye(6))

                        # Map Cartesian velocity to joint velocity
                        cart_vel = action[:6]
                        joint_vel = J_pseudo @ cart_vel

                        # Integrate to get position change
                        self._mock_desired_joint_positions += joint_vel * self._control_interval

                except Exception as e:
                    logger.warning(f"PyBullet IK/Jacobian computation failed: {e}")
                    # Fall back to simple approximation
                    self._update_mock_state_simple_cartesian(action, controller_cfg)
            else:
                # No PyBullet available, use simple approximation
                self._update_mock_state_simple_cartesian(action, controller_cfg)

            # Apply joint limits
            joint_limits = [
                (-2.8973, 2.8973),  # Joint 1
                (-1.7628, 1.7628),  # Joint 2
                (-2.8973, 2.8973),  # Joint 3
                (-3.0718, -0.0698), # Joint 4
                (-2.8973, 2.8973),  # Joint 5
                (-0.0175, 3.7525),  # Joint 6
                (-2.8973, 2.8973),  # Joint 7
            ]
            for i, (low, high) in enumerate(joint_limits):
                self._mock_desired_joint_positions[i] = np.clip(
                    self._mock_desired_joint_positions[i], low, high
                )

    def preprocess(self):
        if self.mock_mode:
            # In mock mode, just reset gripper to open position
            self._mock_gripper_width = 0.08
            logger.debug("Mock mode: Gripper reset to open position")
            return

        gripper_control_msg = franka_controller_pb2.FrankaGripperControlMessage()
        move_msg = franka_controller_pb2.FrankaGripperMoveMessage()
        move_msg.width = 0.08
        move_msg.speed = 0.1
        gripper_control_msg.control_msg.Pack(move_msg)

        logger.debug("Moving Command")
        self._gripper_publisher.send(gripper_control_msg.SerializeToString())

        for _ in range(20):
            dummy_msg = franka_controller_pb2.FrankaDummyControllerMessage()
            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.controller_type = (
                franka_controller_pb2.FrankaControlMessage.ControllerType.NO_CONTROL
            )

            control_msg.control_msg.Pack(dummy_msg)
            control_msg.timeout = 0.2
            control_msg.termination = False

            msg_str = control_msg.SerializeToString()
            self._publisher.send(msg_str)
            time.sleep(0.05)

        logger.debug("Preprocess fnished")

    def control(
        self,
        controller_type: str,
        action: Union[np.ndarray, list],
        controller_cfg: dict = None,
        termination: bool = False,
    ):
        """A function that controls every step on the policy level.

        Args:
            controller_type (str): The type of controller used in this step.
            action (Union[np.ndarray, list]): The action command for the controller.
            controller_cfg (dict, optional): Controller configuration that corresponds to the first argument`controller_type`. Defaults to None.
            termination (bool, optional): If set True, the control will be terminated. Defaults to False.
        """
        # logger.info(f"Raw action received by control: {action}")
        action = np.array(action)
        if self.last_time == None:
            self.last_time = time.time_ns()
        elif not termination:
            # Control the policy frequency if not terminated.
            current_time = time.time_ns()
            remaining_time = self._control_interval - (
                current_time - self.last_time
            ) / (10**9)
            if 0.0001 < remaining_time < self._control_timeout:
                time.sleep(remaining_time)
            self.last_time = time.time_ns()

        if self._last_controller_type != controller_type:
            self.preprocess()
            self._last_controller_type = controller_type

        controller_cfg = verify_controller_config(
            controller_cfg, use_default=True)

        state_estimator_msg = franka_controller_pb2.FrankaStateEstimatorMessage()
        state_estimator_msg.is_estimation = (
            controller_cfg.state_estimator_cfg.is_estimation
        )
        state_estimator_msg.estimator_type = (
            franka_controller_pb2.FrankaStateEstimatorMessage.EstimatorType.EXPONENTIAL_SMOOTHING_ESTIMATOR
        )
        exponential_estimator = franka_controller_pb2.ExponentialSmoothingConfig()
        exponential_estimator.alpha_q = controller_cfg.state_estimator_cfg.alpha_q
        exponential_estimator.alpha_dq = controller_cfg.state_estimator_cfg.alpha_dq
        exponential_estimator.alpha_eef = controller_cfg.state_estimator_cfg.alpha_eef
        state_estimator_msg.config.Pack(exponential_estimator)

        if controller_type == "OSC_POSE":
            assert controller_cfg is not None

            osc_msg = franka_controller_pb2.FrankaOSCPoseControllerMessage()
            osc_msg.translational_stiffness[:] = controller_cfg.Kp.translation
            osc_msg.rotational_stiffness[:] = controller_cfg.Kp.rotation

            osc_config = franka_controller_pb2.FrankaOSCControllerConfig()

            osc_config.residual_mass_vec[:] = controller_cfg.residual_mass_vec
            osc_msg.config.CopyFrom(osc_config)
            action[0:3] *= controller_cfg.action_scale.translation
            action[3: self.last_gripper_dim] *= controller_cfg.action_scale.rotation
            # logger.info(f"Final OSC action: {action}")

            self._history_actions.append(action)
            goal = action_to_osc_pose_goal(
                action, is_delta=controller_cfg.is_delta)
            # Note that goal does not contain gripper information
            osc_msg.goal.CopyFrom(goal)

            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.controller_type = (
                franka_controller_pb2.FrankaControlMessage.ControllerType.OSC_POSE
            )
            control_msg.traj_interpolator_type = TRAJ_INTERPOLATOR_MAPPING[
                controller_cfg.traj_interpolator_cfg.traj_interpolator_type
            ]
            control_msg.traj_interpolator_time_fraction = (
                controller_cfg.traj_interpolator_cfg["time_fraction"]
            )
            control_msg.control_msg.Pack(osc_msg)
            control_msg.timeout = 0.2
            control_msg.termination = termination

            control_msg.state_estimator_msg.CopyFrom(state_estimator_msg)

            if self.mock_mode:
                # In mock mode, update internal state instead of sending message
                self._update_mock_state_from_action(controller_type, action, controller_cfg)
            else:
                msg_str = control_msg.SerializeToString()
                self._publisher.send(msg_str)

        elif controller_type == "OSC_POSITION":
            assert controller_cfg is not None

            osc_msg = franka_controller_pb2.FrankaOSCPoseControllerMessage()
            osc_msg.translational_stiffness[:] = controller_cfg.Kp.translation
            osc_msg.rotational_stiffness[:] = controller_cfg.Kp.rotation

            osc_config = franka_controller_pb2.FrankaOSCControllerConfig()
            osc_config.residual_mass_vec[:] = controller_cfg.residual_mass_vec
            osc_msg.config.CopyFrom(osc_config)

            action[0:3] *= controller_cfg.action_scale.translation
            action[3: self.last_gripper_dim] *= controller_cfg.action_scale.rotation

            self._history_actions.append(action)

            goal = action_to_osc_pose_goal(
                action, is_delta=controller_cfg.is_delta)
            osc_msg.goal.CopyFrom(goal)
            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.controller_type = (
                franka_controller_pb2.FrankaControlMessage.ControllerType.OSC_POSITION
            )
            control_msg.traj_interpolator_type = TRAJ_INTERPOLATOR_MAPPING[
                controller_cfg.traj_interpolator_cfg.traj_interpolator_type
            ]
            control_msg.traj_interpolator_time_fraction = (
                controller_cfg.traj_interpolator_cfg["time_fraction"]
            )
            control_msg.control_msg.Pack(osc_msg)
            control_msg.timeout = 0.2
            control_msg.termination = termination

            control_msg.state_estimator_msg.CopyFrom(state_estimator_msg)

            if self.mock_mode:
                # In mock mode, update internal state instead of sending message
                self._update_mock_state_from_action(controller_type, action, controller_cfg)
            else:
                msg_str = control_msg.SerializeToString()
                self._publisher.send(msg_str)

        elif controller_type == "OSC_YAW":
            assert controller_cfg is not None

            osc_msg = franka_controller_pb2.FrankaOSCPoseControllerMessage()
            osc_msg.translational_stiffness[:] = controller_cfg.Kp.translation
            osc_msg.rotational_stiffness[:] = controller_cfg.Kp.rotation

            osc_config = franka_controller_pb2.FrankaOSCControllerConfig()
            osc_config.residual_mass_vec[:] = controller_cfg.residual_mass_vec
            osc_msg.config.CopyFrom(osc_config)

            action[0:3] *= controller_cfg.action_scale.translation
            action[3: self.last_gripper_dim] *= controller_cfg.action_scale.rotation

            self._history_actions.append(action)

            goal = action_to_osc_pose_goal(
                action, is_delta=controller_cfg.is_delta)
            osc_msg.goal.CopyFrom(goal)
            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.controller_type = (
                franka_controller_pb2.FrankaControlMessage.ControllerType.OSC_YAW
            )
            control_msg.traj_interpolator_type = TRAJ_INTERPOLATOR_MAPPING[
                controller_cfg.traj_interpolator_cfg.traj_interpolator_type
            ]
            control_msg.traj_interpolator_time_fraction = (
                controller_cfg.traj_interpolator_cfg["time_fraction"]
            )
            control_msg.control_msg.Pack(osc_msg)
            control_msg.timeout = 0.2
            control_msg.termination = termination

            control_msg.state_estimator_msg.CopyFrom(state_estimator_msg)

            if self.mock_mode:
                # In mock mode, update internal state instead of sending message
                self._update_mock_state_from_action(controller_type, action, controller_cfg)
            else:
                msg_str = control_msg.SerializeToString()
                self._publisher.send(msg_str)

        elif controller_type == "JOINT_POSITION":
            assert controller_cfg is not None
            assert len(action) == 7 + 1

            joint_pos_msg = franka_controller_pb2.FrankaJointPositionControllerMessage()
            joint_pos_msg.speed_factor = 0.1
            goal = action_to_joint_pos_goal(
                action, is_delta=controller_cfg.is_delta)

            joint_pos_msg.goal.CopyFrom(goal)

            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.controller_type = (
                franka_controller_pb2.FrankaControlMessage.ControllerType.JOINT_POSITION
            )
            control_msg.traj_interpolator_type = (
                franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.SMOOTH_JOINT_POSITION
            )
            control_msg.traj_interpolator_time_fraction = (
                controller_cfg.traj_interpolator_cfg["time_fraction"]
            )
            control_msg.control_msg.Pack(joint_pos_msg)
            control_msg.timeout = 0.2
            control_msg.termination = termination

            control_msg.state_estimator_msg.CopyFrom(state_estimator_msg)

            if self.mock_mode:
                # In mock mode, update internal state instead of sending message
                self._update_mock_state_from_action(controller_type, action, controller_cfg)
            else:
                msg_str = control_msg.SerializeToString()
                self._publisher.send(msg_str)

        elif controller_type == "JOINT_IMPEDANCE":
            assert controller_cfg is not None
            assert len(action) == 7 + 1

            joint_impedance_msg = (
                franka_controller_pb2.FrankaJointImpedanceControllerMessage()
            )
            goal = action_to_joint_pos_goal(
                action, is_delta=controller_cfg.is_delta)
            joint_impedance_msg.goal.CopyFrom(goal)

            joint_impedance_msg.kp[:] = controller_cfg.joint_kp
            joint_impedance_msg.kd[:] = controller_cfg.joint_kd

            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.controller_type = (
                franka_controller_pb2.FrankaControlMessage.ControllerType.JOINT_IMPEDANCE
            )
            control_msg.traj_interpolator_type = TRAJ_INTERPOLATOR_MAPPING[
                controller_cfg.traj_interpolator_cfg.traj_interpolator_type
            ]
            control_msg.traj_interpolator_time_fraction = (
                controller_cfg.traj_interpolator_cfg["time_fraction"]
            )
            control_msg.control_msg.Pack(joint_impedance_msg)
            control_msg.timeout = 0.2
            control_msg.termination = termination

            control_msg.state_estimator_msg.CopyFrom(state_estimator_msg)

            if self.mock_mode:
                # In mock mode, update internal state instead of sending message
                self._update_mock_state_from_action(controller_type, action, controller_cfg)
            else:
                msg_str = control_msg.SerializeToString()
                self._publisher.send(msg_str)

        elif controller_type == "CARTESIAN_VELOCITY":
            assert controller_cfg is not None

            cartesian_velocity_msg = (
                franka_controller_pb2.FrankaCartesianVelocityControllerMessage()
            )

            action[0:3] *= controller_cfg.action_scale.translation
            action[3: self.last_gripper_dim] *= controller_cfg.action_scale.rotation

            logger.debug(f"OSC action: {np.round(action, 3)}")

            self._history_actions.append(action)
            goal = action_to_cartesian_velocity(
                action, is_delta=controller_cfg.is_delta
            )
            cartesian_velocity_msg.goal.CopyFrom(goal)

            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.controller_type = (
                franka_controller_pb2.FrankaControlMessage.ControllerType.CARTESIAN_VELOCITY
            )
            control_msg.traj_interpolator_type = TRAJ_INTERPOLATOR_MAPPING[
                controller_cfg.traj_interpolator_cfg.traj_interpolator_type
            ]
            control_msg.traj_interpolator_time_fraction = (
                controller_cfg.traj_interpolator_cfg["time_fraction"]
            )
            control_msg.control_msg.Pack(cartesian_velocity_msg)
            control_msg.timeout = 0.2
            control_msg.termination = termination

            control_msg.state_estimator_msg.CopyFrom(state_estimator_msg)

            if self.mock_mode:
                # In mock mode, update internal state instead of sending message
                self._update_mock_state_from_action(controller_type, action, controller_cfg)
            else:
                msg_str = control_msg.SerializeToString()
                self._publisher.send(msg_str)

        if self.has_gripper:
            self.gripper_control(action[self.last_gripper_dim])

        if self.use_visualizer and len(self._state_buffer) > 0:
            self.visualizer.update(
                joint_positions=np.array(self._state_buffer[-1].q))

    def gripper_control(self, action: float):
        """Control the gripper

        Args:
            action (float): The control command for Franka gripper. Currently assuming scalar control commands.
            action should be between 0 and 1
        """

        if self.mock_mode:
            # In mock mode, directly update gripper width
            self._mock_gripper_width = 0.08 * np.clip(action, 0.0, 1.0)
            logger.debug(f"Mock mode: Gripper width set to {self._mock_gripper_width}")
        else:
            gripper_control_msg = franka_controller_pb2.FrankaGripperControlMessage()

            # action 0-> 1 : Grasp
            # action 1-> 0 : Release

            # TODO (Yifeng): Test if sending grasping or gripper directly
            # will stop executing the previous command
            # if action < 0.0:  #  and self.last_gripper_action == 1):
            move_msg = franka_controller_pb2.FrankaGripperMoveMessage()
            move_msg.width = 0.08 * np.clip(action, 0.0, 1.0)
            move_msg.speed = 0.1
            gripper_control_msg.control_msg.Pack(move_msg)

            # logger.debug("Gripper actuating to width: {}".format(move_msg.width / 0.08))

            self._gripper_publisher.send(gripper_control_msg.SerializeToString())
        # elif action >= 0.0:  #  and self.last_gripper_action == 0:
        #     grasp_msg = franka_controller_pb2.FrankaGripperGraspMessage()
        #     grasp_msg.width = -0.01
        #     grasp_msg.speed = 0.5
        #     grasp_msg.force = 30.0
        #     grasp_msg.epsilon_inner = 0.08
        #     grasp_msg.epsilon_outer = 0.08

        #     gripper_control_msg.control_msg.Pack(grasp_msg)

        #     logger.debug("Gripper closing")

        #     self._gripper_publisher.send(gripper_control_msg.SerializeToString())
        self.last_gripper_action = action

    def close(self):
        self.termination = True
        self._state_sub_thread.join(1.0)
        if self.mock_mode:
            logger.info("Mock mode: Closing FrankaInterface")

    @property
    def last_eef_pose(self) -> np.ndarray:
        """_summary_

        Returns:
            np.ndarray: The 4x4 homogeneous matrix of end effector pose.
        """
        if self.state_buffer_size == 0:
            return None
        return np.array(self._state_buffer[-1].O_T_EE).reshape(4, 4).transpose()

    @property
    def last_eef_rot_and_pos(self) -> Tuple[np.ndarray, np.ndarray]:
        """_summary_

        Returns:
            Tuple[np.ndarray, np.ndarray]: (eef_rot, eef_pos), eef_rot in rotation matrix, eef_pos in 3d vector.
        """
        if self.state_buffer_size == 0:
            return None, None
        O_T_EE = np.array(
            self._state_buffer[-1].O_T_EE).reshape(4, 4).transpose()
        return O_T_EE[:3, :3], O_T_EE[:3, 3:]

    @property
    def last_eef_quat_and_pos(self) -> Tuple[np.ndarray, np.ndarray]:
        """_summary_

        Returns:
            Tuple[np.ndarray, np.ndarray]: (eef_quat, eef_pos), eef_quat in quaternion (xyzw), eef_pos in 3d vector.
        """
        if self.state_buffer_size == 0:
            return None, None
        O_T_EE = np.array(
            self._state_buffer[-1].O_T_EE).reshape(4, 4).transpose()
        return transform_utils.mat2quat(O_T_EE[:3, :3]), O_T_EE[:3, 3:]

    def check_nonzero_configuration(self) -> bool:
        """Check nonzero configuration.

        Returns:
            bool: The boolean variable that indicates if the reading of robot joint configuration is non-zero.
        """
        if np.max(np.abs(np.array(self._state_buffer[-1].O_T_EE))) < 1e-3:
            return False
        return True

    def reset(self):
        """Reset internal states of FrankaInterface and clear buffers. Useful when you run multiple episodes in a single python interpretor process."""
        self._state_buffer = []
        self._state_buffer_idx = 0

        self._gripper_state_buffer = []
        self._gripper_buffer_idx = 0

        self.counter = 0
        self.termination = False

        self.last_time = None
        self.last_gripper_dim = -1
        self.last_gripper_action = 0
        self.last_gripper_command_counter = 0
        self._history_actions = []

    @property
    def received_states(self):
        return len(self._state_buffer) > 0

    @property
    def last_q(self) -> np.ndarray:
        if self.state_buffer_size == 0:
            return None
        return np.array(self._state_buffer[-1].q)

    @property
    def last_q_d(self) -> np.ndarray:
        if self.state_buffer_size == 0:
            return None
        return np.array(self._state_buffer[-1].q_d)

    @property
    def last_gripper_q(self) -> np.ndarray:
        if self.gripper_state_buffer_size == 0:
            return None
        return np.array(self._gripper_state_buffer[-1].width)

    @property
    def last_dq(self) -> np.ndarray:
        if self.state_buffer_size == 0:
            return None
        return np.array(self._state_buffer[-1].dq)

    @property
    def last_state(self):
        """Default state"""
        if self.state_buffer_size == 0:
            return None
        return self._state_buffer[-1]

    @property
    def last_pose(self):
        if self.state_buffer_size == 0:
            return None
        return np.array(self._state_buffer[-1].O_T_EE).reshape(4, 4).transpose()

    @property
    def state_buffer_size(self) -> int:
        return len(self._state_buffer)

    @property
    def gripper_state_buffer_size(self) -> int:
        return len(self._gripper_state_buffer)

    @property
    def ip(self) -> str:
        return self._ip

    @property
    def pub_port(self) -> int:
        return self._pub_port

    @property
    def sub_port(self) -> int:
        return self._sub_port

    @property
    def gripper_pub_port(self) -> int:
        return self._gripper_pub_port

    @property
    def gripper_sub_port(self) -> int:
        return self._gripper_sub_port

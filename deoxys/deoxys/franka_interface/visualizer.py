import pathlib
from abc import ABC, abstractmethod

import numpy as np
import pybullet

FILE_PATH = pathlib.Path(__file__).parent.absolute()


def visualizer_factory(backend, *args, **kwargs):
    if backend == "pybullet":
        return PybulletVisualizer(*args, **kwargs)
    else:
        raise NotImplementedError


class Visualizer:
    def __init__(self):
        pass

    @abstractmethod
    def render(self):
        raise NotImplementedError


class PybulletVisualizer(Visualizer):
    def __init__(self):
        super().__init__()
        self._uid = pybullet.connect(pybullet.GUI)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 1)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)

        # Store debug line IDs for coordinate frames to avoid accumulation
        self._coordinate_frame_lines = []

        position = [0.0, 0.0, 0.0]
        quaternion = [0.0, 0.0, 0.0, 1.0]
        scale = 1.0

        pybullet.loadURDF(
            fileName=str(FILE_PATH) + "/robot_models/planes/plane_ceramic.urdf",
            basePosition=[0.0, 0.0, 0.0],
            baseOrientation=[0.0, 0.0, 0.0, 1.0],
            globalScaling=1.0,
            useFixedBase=True,
            physicsClientId=self._uid,
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        )

        self.robot_uid = pybullet.loadURDF(
            fileName=str(FILE_PATH) + "/robot_models/panda/panda_marker.urdf",
            basePosition=position,
            baseOrientation=quaternion,
            globalScaling=scale,
            useFixedBase=True,
            physicsClientId=self._uid,
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        )
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)
        pybullet.resetDebugVisualizerCamera(
            cameraDistance=1.4,
            cameraYaw=45,
            cameraPitch=-45,
            cameraTargetPosition=np.array(position) + np.array([0.0, 0.0, 0.3]),
            physicsClientId=self._uid,
        )

        self.num_arm_joints = 7

        # The marker is the last link of the URDF file
        self.marker_ind = (
            pybullet.getNumJoints(self.robot_uid, physicsClientId=self._uid) - 1
        )

        # End-effector link index (same as FrankaInterface)
        self.ee_link_index = 8

    def render(self):
        # Render
        pybullet.stepSimulation(physicsClientId=self._uid)

    def update(self, joint_positions, vis_gripper=False, draw_ee_frame=False):
        # Update states
        num_robot_joints = 0
        if not vis_gripper:
            assert len(joint_positions) == self.num_arm_joints
            num_robot_joints = self.num_arm_joints

        for joint_ind in range(self.num_arm_joints):
            pybullet.resetJointState(
                bodyUniqueId=self.robot_uid,
                jointIndex=joint_ind,
                targetValue=joint_positions[joint_ind],
                physicsClientId=self._uid,
            )

        # Draw end-effector coordinate frame if requested
        if draw_ee_frame:
            self.draw_ee_coordinate_frame()

    def get_marker_pose(self):
        marker_link_state = pybullet.getLinkState(
            bodyUniqueId=self.robot_uid,
            jointIndex=i,
            targetValue=self.marker_ind,
            physicsClientId=self._uid,
        )
        marker_pos, marker_quat = marker_link_state[0], marker_link_state[1]
        return (marker_pos, marker_quat)

    def draw_coordinate_frame(
        self, position, orientation, axis_length=0.1, line_width=3
    ):
        """Draw a coordinate frame (X, Y, Z axes) at the specified pose.

        Args:
            position: [x, y, z] position of the frame origin
            orientation: [x, y, z, w] quaternion orientation
            axis_length: Length of each axis line
            line_width: Width of the axis lines

        Returns:
            List of debug line IDs (can be used to remove lines later)
        """
        # Remove old coordinate frame lines to avoid accumulation
        for line_id in self._coordinate_frame_lines:
            try:
                pybullet.removeUserDebugItem(line_id, physicsClientId=self._uid)
            except:
                pass  # Line might already be removed
        self._coordinate_frame_lines = []

        # Convert quaternion to rotation matrix
        rot_matrix = pybullet.getMatrixFromQuaternion(orientation)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        # Define axis directions
        axes = [
            (np.array([1, 0, 0]), [1, 0, 0]),  # X axis - Red
            (np.array([0, 1, 0]), [0, 1, 0]),  # Y axis - Green
            (np.array([0, 0, 1]), [0, 0, 1]),  # Z axis - Blue
        ]

        line_ids = []
        for axis_dir, color in axes:
            # Rotate axis direction by the frame orientation
            axis_end = position + rot_matrix @ (axis_dir * axis_length)

            # Draw the axis line
            line_id = pybullet.addUserDebugLine(
                lineFromXYZ=position,
                lineToXYZ=axis_end.tolist(),
                lineColorRGB=color,
                lineWidth=line_width,
                physicsClientId=self._uid,
            )
            line_ids.append(line_id)

        # Store the new line IDs
        self._coordinate_frame_lines = line_ids

        return line_ids

    def draw_ee_coordinate_frame(self, axis_length=0.1, line_width=3):
        """Draw the end-effector coordinate frame from the visualizer's robot.

        This method gets the end-effector pose from the robot loaded in the visualizer
        and draws the coordinate frame at that location.

        Args:
            axis_length: Length of each axis line
            line_width: Width of the axis lines

        Returns:
            List of debug line IDs
        """
        # Get end-effector link state from the visualizer's robot
        link_state = pybullet.getLinkState(
            self.robot_uid,
            self.ee_link_index,
            computeForwardKinematics=True,
            physicsClientId=self._uid,
        )

        pos = np.array(link_state[4])  # World position
        orn = np.array(link_state[5])  # World orientation (quaternion)

        # Draw the coordinate frame
        return self.draw_coordinate_frame(pos, orn, axis_length, line_width)

# -*- coding: utf-8 -*-
import rclpy
from tf2_ros import TransformException
from rclpy.node import Node
from geometry_msgs.msg import Pose as PoseMsg
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import numpy as np
from typing import List, Dict, Any

from .transform import Affine
from .robot import RobotClient
from .camera import Camera


# TODO use manipulation_tasks Scene, Robot and Camera (Sensor) protocols
#  or maybe actually not Scene because of inapplicable functions 


class RosScene(Node):
    """
    TODO description
    """

    def __init__(self, is_simulation: bool = False, t_bounds: np.ndarray = None):
        """
        TODO docstring

        Returns
        -------
        None.

        """
        super().__init__("ros_scene")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pixel_size = 0.00315
        self.grasp_offset = Affine(translation=[0.0, 0, 0.602])
        self.robot = RobotClient(node=self, is_simulation=is_simulation)

        # TODO get real values for resolution, ...
        self.resolution = (480, 640)
        self.intrinsics = (450., 0, 320., 0, 450., 240., 0, 0, 1)
        self.depth_range = (0.01, 10.)
        camera_poses = [Affine()]  # TODO change
        fixed = True
        self.cameras = [
            Camera(camera_poses[0], self.resolution, self.intrinsics,
                   self.depth_range, fixed=fixed)]

        # TODO how to get real t_bounds and r_bounds
        if t_bounds is not None:
            self.t_bounds = np.array([[0.25, 0.75], [-0.5, 0.5], [0, 0]])
        else:
            self.t_bounds = t_bounds
        self.r_bounds = np.array([[0, 0], [0, 0], [0, 2 * np.pi]])

    def get_observation(self, poses: List = None) -> List[Dict[str, Any]]:
        """
        ...
        TODO docstring

        Returns
        -------
        List[Dict[str, Any]]
            DESCRIPTION.

        """
        observations = list()
        if poses is not None:
            # TODO find better solution
            camera = None
            for cam in self.cameras:
                if not cam.fixed:
                    camera = cam
                    break
            if camera is not None:
                for pose in poses:
                    self.robot.ptp(pose)  # TODO add offset to set camera on robot to pose instead of robot?
                    observations.append(self.camera.get_observation())
        else:
            for camera in self.cameras:
                if camera.fixed:
                    observations.append(self.camera.get_observation())
        return observations

    def spawn_coordinate_frame(self, pose: Affine):
        # TODO remove function after testing (not applicable to real scene)
        # TODO add content
        ...

    def clean(self):
        # TODO remove function after testing (not applicable to real scene)
        # TODO add content
        ...

    def shutdown(self):
        self.destroy_node()

    def get_transform(self, to_frame_rel='cell_link', from_frame_rel='camera_color_optical_frame'):
        counter = 0
        has_transform = False
        transform = None
        while (not has_transform) and counter < 10:
            counter += 1
            try:
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(
                    to_frame_rel, from_frame_rel, now)
                trans = trans.transform
                transform = Affine(
                    [trans.translation.x, trans.translation.y, trans.translation.z],
                    [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])
                has_transform = self.tf_buffer.can_transform(
                    to_frame_rel, from_frame_rel, now)
                print("Has transform: ", has_transform, counter)
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        return transform

    def get_current_pose(self):
        transform_tcp_link = self.get_transform('cell_link', 'tcp_link')
        curret_pose = PoseMsg()
        curret_pose.position.x = transform_tcp_link.translation[0]
        curret_pose.position.y = transform_tcp_link.translation[1]
        curret_pose.position.z = transform_tcp_link.translation[2]
        curret_pose.orientation.x = transform_tcp_link.quat[0]
        curret_pose.orientation.y = transform_tcp_link.quat[1]
        curret_pose.orientation.z = transform_tcp_link.quat[2]
        curret_pose.orientation.w = transform_tcp_link.quat[3]
        return RobotClient.pose_to_affine(curret_pose)

    def get_current_pixel_pose(self):
        current_pose = self.get_current_pose()

        u_v = position_to_pixel(current_pose.translation, self.t_bounds, self.pixel_size)

        return u_v[0], u_v[1], current_pose.rpy[2]


def position_to_pixel(position, bounds, pixel_size):
    # TODO maybe clip
    u = int(np.round((position[0] - bounds[0, 0]) / pixel_size))
    v = int(np.round((position[1] - bounds[1, 0]) / pixel_size))
    return u, v
# TODO register RosScene (and find better name...)

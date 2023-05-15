# -*- coding: utf-8 -*-
from rclpy.node import Node
from typing import Tuple, Dict, Union

from .transform import Affine


# TODO use manipulation_tasks.Camera


class Camera(Node):
    """
    TODO description
    """
    def __init__(
            self,
            pose: Affine,
            resolution: Tuple[int, int],
            intrinsics: Tuple[float, float, float, 
                              float, float, float, 
                              float, float, float],
            depth_range: Tuple[float, float],
            fixed: bool) -> None:
        """
        TODO docstring
        """
        # TODO actually use pose, resolution, ...
        self.pose = pose
        self.resolution = resolution
        self.intrinsics = intrinsics
        self.depth_range = depth_range
        self.fixed = fixed
        
    def get_config(self) -> Dict[str, Union[
            Affine, 
            Tuple[int, int],
            Tuple[float, float, float, float, float, float, float, float, float],
            Tuple[float, float]]]:
        """
        TODO docstring

        Returns
        -------
        (Dict[str, Union[Affine, Tuple[int, int], Tuple[float, float, float, 
        float, float, float, float, float, float], Tuple[float, float]]])
            DESCRIPTION.

        """
        # TODO no need to implement function as soon as manipulation_tasks.Camera inherited
        config = {
            "pose": self.pose,
            "resolution": self.resolution,
            "intrinsics": self.intrinsics,
            "depth_range": self.depth_range}
        return config
    
    def get_observation(self):
        """
        TODO docstring

        Returns
        -------
        None.

        """
        # TODO implement
        ...
        
    def read_data(self):
        # TODO no need to implement function as soon as manipulation_tasks.Camera inherited
        return self.get_observation()

        

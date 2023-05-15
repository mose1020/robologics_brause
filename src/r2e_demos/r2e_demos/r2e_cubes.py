import rclpy
from typing import List
from ros_environment.robot import RobotClient
from ros_environment.transform import Affine


class CubesDemo():
    """ Class to demonstrate the KUKA ready2_educate cubes example with ros2.
    """
    def __init__(
            self,
            is_simulation: bool = False,
            home_pose: Affine = Affine(
                (0.122, -0.052, 1.426), 
                (0.004, 0.860, 0.007, 0.510)),
            ramp_bottom=Affine(
                (-0.114, -0.484, 1.050), 
                (0.009, 0.932, 0.001, -0.362)),
            ramp_top=Affine(
                (-0.250, -0.484, 1.193), 
                (0.009, 0.932, 0.001, -0.362))) -> None:
        """ Initialize the cubes demo class.

        Parameters
        ----------
        is_simulation : bool, default=False
            Defines if the demo runs on the real robot or in simulation.
        home_pose : Affine, default=Affine((0.122, -0.052, 1.426), (0.004, 0.860, 0.007, 0.510)
            The robot's cartesian home pose.
        ramp_top : Affine, default=Affine((-0.250, -0.484, 1.193), (0.009, 0.932, 0.001, -0.362))
            The robot's cartesian pose to reach the top of the cubes ramp. The default pose
            is correct if the ramp is positioned as follows: the mounting holes that are
            nearer to the ramp respectively are used. The mounting holes are at positions
            B13 (further away from ramp) and B9 (directly next to ramp).
            ramp-side (hole B9) (unused hole) space (hole B13) (unused hole) vertical-column-side
        ramp_bottom : Affine, default=Affine((-0.114, -0.484, 1.050), (0.009, 0.932, 0.001, -0.362))
            The robots' cartesian pose to reach the cubes at the bottom of the ramp.
            The default pose is correct if the ramp is positioned as described in "ramp_top"
            parameter description.

        """
        self.robot = RobotClient(is_simulation=is_simulation)
        self.robot.home_pose=home_pose
        self.ramp_top = ramp_top
        self.ramp_bottom = ramp_bottom
        
    def run_demo(
            self, 
            matrix_center: Affine = Affine(
                (0.123, -0.024, 1.026), 
                (0.008, 1.000, 0.004, 0.013)),
            distance_x: float = 0.04,
            distance_y: float = 0.06) -> None:
        """ Run the cubes demo: build a 3x3 matrix of cubes and tidy up afterwards.
        
        Parameters
        ----------
        matrix_center : Affine, default=Affine((0.123, -0.024, 1.026), (0.008, 1.000, 0.004, 0.013))
            The pose to position the central cube of the matrix.
        matrix_distance : float, default=0.074
            The distance (in meters) between the cube centers in the matrix. Be careful 
            to choose a value that leaves enough space for the opened gripper.

        """
        # TODO change matrix center default pose
        self.robot.home()
        matrix = self.get_matrix_poses(matrix_center, distance_x=distance_x, distance_y=distance_y)
        self.set_up_cubes(matrix)
        self.robot.home()
        self.tidy_up_cubes(matrix)
        self.robot.home()
        self.shutdown_demo()
    
    def shutdown_demo(self) -> None:
        """ Finish up the demo by destroying the RobotClient node.
            
        """
        self.robot.destroy_node()
        
    @staticmethod
    def get_matrix_poses(center_pose: Affine, distance_x: float, distance_y: float) -> List[Affine]:
        """ Get each pose for the 3x3 cube matrix.

        Parameters
        ----------
        center_pose : Affine
            The pose to position the central cube of the matrix.
        distance : float
            The distance (in meters) between the cube centers in the matrix.

        Returns
        -------
        List[Affine]
            List of the cube matrix poses.

        """
        poses = list()
        for x_direction_dist in [-distance_x, 0, distance_x]:
            for y_direction_dist in [-distance_y, 0, distance_y]:
                copy_pose = Affine(center_pose.translation, center_pose.quat)
                copy_pose.translation[0] = copy_pose.translation[0] + x_direction_dist
                copy_pose.translation[1] = copy_pose.translation[1] + y_direction_dist
                poses.append(copy_pose)
        return poses
    
    def get_pre_pose(self, pose: Affine, distance: float = 0.05) -> Affine:
        """ Calculate the pre-pick and pre-place pose with the given distance 
        to the given pose.

        Parameters
        ----------
        pose : Affine
            The final pick/place pose.
        distance : float, default=0.05
            Distance between pre-pose and final pose.

        Returns
        -------
        pre_pose : TYPE
            The pre-pose (pre-pick/pre-place pose).

        """
        pre_pose_transform = Affine(translation=(0, 0, -distance))
        pre_pose = pose * pre_pose_transform
        return pre_pose
    
    def pick(self, pick_pose: Affine) -> None:
        """ Pick process: open the gripper, move to pre-pick pose (ptp),
        move to pick pose (lin), close the gripper, move back to pre-pick pose 
        (lin).
        
        Parameters
        ----------
        pick_pose : Affine
            The pick pose.

        """
        self.robot.open_gripper()
        pre_pick = self.get_pre_pose(pick_pose, distance=0.05)
        self.robot.ptp(pre_pick)
        self.robot.lin(pick_pose)
        self.robot.close_gripper()
        self.robot.lin(pre_pick)
        
    def place(self, place_pose: Affine) -> None:
        """ Place process: move to pre-place pose (ptp), move to place pose 
        (lin), open gripper, move to pre-pace (lin).

        Parameters
        ----------
        place_pose : Affine
            The place pose.

        """
        pre_place = self.get_pre_pose(place_pose, distance=0.05)
        self.robot.ptp(pre_place)
        self.robot.lin(place_pose)
        self.robot.open_gripper()
        self.robot.lin(pre_place)
        
    def set_up_cubes(self, pose_matrix: List[Affine]) -> None:
        """ Set up cubes from the ramp in a 3x3 matrix in front of the robot.

        Parameters
        ----------
        pose_matrix : List[Affine]
            The place poses for the cubes in the matrix.

        """
        for place_pose in pose_matrix:
            self.pick(self.ramp_bottom)
            self.place(place_pose)
            
    def tidy_up_cubes(self, pose_matrix: List[Affine]) -> None:
        """ Tidy up the cubes from a 3x3 matrix and put them back on the ramp.

        Parameters
        ----------
        pose_matrix : List[Affine]
            The pick poses for the cubes in the matrix.

        """
        for pick_pose in pose_matrix:
            self.pick(pick_pose)
            self.place(self.ramp_top)        

        
def main(args=None):
    # initialize ros communications for a given context
    rclpy.init(args=args)
    
    """ 
    The example only works correctly if the ramp with the cubes is positioned
     as follows: The mounting holes that are nearer to the ramp respectively 
     are used. The mounting holes are fixed at positions B9 (directly next to 
     ramp) and B13 (further away from ramp).
    --> ramp-side (hole B9) (unused hole) space (hole B13) (unused hole) vertical-column-side
    To use the ramp at another position, please change the ramp_top and 
     ramp_bottom parameters when initialising the CubesDemo object.

    To follow the KUKA cubes tutorial worksheet please position the frame holes 
     at H5, H1, N5, N1.
    The cube matrix position can be changed as well if desired by setting the
     matrix_center and matrix_distance parameters of the run_demo function as
     described in its docstring.

    To test the demo on the real robot with opening and closing the gripper, please set
     is_simulation=False when initialising the CubesDemo.
    """
    # initialise the cubes demo
    cubes = CubesDemo(
        is_simulation=False,
        home_pose=Affine((0.122, -0.052, 1.426), (0.004, 0.860, 0.007, 0.510)),
        ramp_bottom=Affine((-0.114, -0.484, 1.050), (0.009, 0.932, 0.001, -0.362)),
        ramp_top=Affine((-0.250, -0.484, 1.193), (0.009, 0.932, 0.001, -0.362)))
    # run the cubes demo
    cubes.run_demo(
        matrix_center=Affine((0.123, -0.024, 1.026), (0.008, 1.000, 0.004, 0.013)),
        distance_x=0.04,
        distance_y=0.06)
    
    # shutdown previously initialized context
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from ros_environment.scene import RobotClient
from ros_environment.transform import Affine
import time
class BrausePicker:
    def __init__(
            self,
            is_simulation: bool = False,
            home_pose: Affine = Affine((0.0, -0.05, 1.4), (0.608, 0.608, 0.361, 0.361)),
            ) -> None:
        """ Initialize the cubes demo class.

        Parameters
        ----------
        is_simulation : bool, default=False
            Defines if the demo runs on the real robot or in simulation.
        home_pose : Affine, default=Affine((0.122, -0.052, 1.426), (0.004, 0.860, 0.007, 0.510)
            The robot's cartesian home pose.
        

        """
        self.robot = RobotClient(is_simulation=is_simulation)
        self.robot.home_pose=home_pose
    
    def get_pre_pose(self, pose: Affine, distance: float = 0.1) -> Affine:
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
        #pre_pose_transform = Affine(translation=(0, 0, -distance))
        #pre_pose = pose * pre_pose_transform
        
        # prepose is 10cm above the pick pose
        pre_pose = Affine((pose.translation[0], pose.translation[1], pose.translation[2] + distance), pose.quat)

        return pre_pose
     
    def pick(self, pick_pose: Affine) -> None:
        """ Pick process: activate the vacuumgripper, move to pre-pick pose (ptp),
        move to pick pose (lin), move back to pre-pick pose 
        (lin).
        
        Parameters
        ----------
        pick_pose : Affine
            The pick pose.

        """
        
        pre_pick = self.get_pre_pose(pick_pose, distance=0.1)
        time.sleep(1.0)
        print("pre_pick")
        print(pre_pick)

        print("pick_pose")
        print(pick_pose)

        self.robot.ptp(pre_pick)
        self.robot.close_vacuum_gripper()
        self.robot.lin(pick_pose)
        #maybe a wait is needed here
        self.robot.lin(pre_pick) 

    def move_to_camera(self) -> None:
        #move to camera position via home to avoid collision and entanglement
        self.robot.home()
        self.robot.ptp(Affine((-0.126, 0.270, 1.503), (-0.554, 0.502, -0.436, 0.501)))
        #time.sleep(1.0)   

    def drop_at_slide(self) -> None:
        #drop position
        self.robot.ptp(Affine((0.109, -0.389, 1.221), (0.497, 0.497, 0.503, 0.503)))
        self.robot.open_vacuum_gripper()
        #time.sleep(1.0)

    def home(self) -> None:
        """ Move the robot to its home pose.

        """
        self.robot.home()

    def shutdown(self) -> None:
        """ Finish up the demo by destroying the RobotClient node.
            
        """
        self.robot.destroy_node()

def main(args=None):

    # initialize ros communications for a given context
    rclpy.init(args=args)
    # create a new demo instance
    test_picks = BrausePicker(is_simulation=False)

    # user chooses color and if its available the robot picks it otherwise new color is chosen
    pose_from_camera = Affine((0.024, 0.319, 1.041), (0.444, 0.445, 0.550, 0.550))


    test_picks.home()


    test_picks.pick(pose_from_camera)

    test_picks.home()
    test_picks.drop_at_slide()
    test_picks.home()

    
    test_picks.shutdown()
    # shutdown previously initialized context
    rclpy.shutdown()

if __name__ == '__main__':
    main()
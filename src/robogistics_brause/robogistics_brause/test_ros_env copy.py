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
        """ Pick process: activate the vacuumgripper, move to pre-pick pose (ptp),
        move to pick pose (lin), move back to pre-pick pose 
        (lin).
        
        Parameters
        ----------
        pick_pose : Affine
            The pick pose.

        """
        self.robot.close_vacuum_gripper()
        pre_pick = self.get_pre_pose(pick_pose, distance=0.1)
        self.robot.ptp(pre_pick)
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


def main(args=None):

    # initialize ros communications for a given context
    rclpy.init(args=args)

    robot = RobotClient(is_simulation=False)
    robot.home_pose = Affine((0.122, -0.052, 1.426), (0.004, 0.860, 0.007, 0.510))
    # #drop_pose = [(0.109, -0.389, 1.121), (0.497, 0.497, 0.503, 0.503)]

    #move robot to home pose
    robot.home()
    # print("Finished",finished)

    time.sleep(1)
    #print("StartTime:",time.asctime( time.localtime(time.time()) ))
    
    # # robot ptp movement again
    #print("AfterCloseTime:",time.asctime( time.localtime(time.time()) ))

    #-----------pick sth from R5 on the grid----------------------------------------------
    #move to above position
    robot.lin(Affine((0.024, 0.319, 1.121), (0.444, 0.445, 0.550, 0.550)))
    robot.close_vacuum_gripper()
    time.sleep(0.1)

    # # move down
    robot.lin(Affine((0.024, 0.319, 1.021), (0.444, 0.445, 0.550, 0.550)))
    time.sleep(0.1)

    # #move up again and to the above position
    robot.lin(Affine((0.024, 0.319, 1.121), (0.444, 0.445, 0.550, 0.550)))
    time.sleep(0.1)
    
    #check in camera
    robot.ptp(Affine((-0.126, 0.270, 1.503), (-0.554, 0.502, -0.436, 0.501)))
    time.sleep(0.1)
    # #drop position
    robot.ptp(Affine((0.109, -0.389, 1.221), (0.497, 0.497, 0.503, 0.503)))
    # time.sleep(5)

    
    
    # time.sleep(1)
    # print("AfterSleep2:",time.asctime( time.localtime(time.time()) ))
    robot.open_vacuum_gripper()
    time.sleep(1)
    #back to home pose
    robot.home()
    time.sleep(2)
    #print('EndTime:',time.asctime( time.localtime(time.time()) ))
    # # destroy the robot node
    
    robot.destroy_node()
    # shutdown previously initialized context
    rclpy.shutdown()

if __name__ == '__main__':
    main()
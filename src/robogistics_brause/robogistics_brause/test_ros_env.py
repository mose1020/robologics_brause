import rclpy
from ros_environment.scene import RobotClient
from ros_environment.transform import Affine
import time

def main(args=None):
    # initialize ros communications for a given context
    rclpy.init(args=args)

    robot = RobotClient(is_simulation=False)
    time.sleep(2)
    print("StartTime:",time.asctime( time.localtime(time.time()) ))
    robot.close_vacuum_gripper()
    # robot ptp movement again
    print("AfterCloseTime:",time.asctime( time.localtime(time.time()) ))
    robot.home_pose = Affine((0.122, -0.052, 1.426), (0.004, 0.860, 0.007, 0.510))
    
    # move robot to home pose
    robot.home()

    #robot.close_vacuum_gripper()
    # back to home pose
    time.sleep(2)
    print("AfterSleep2:",time.asctime( time.localtime(time.time()) ))
    robot.open_vacuum_gripper()
   
    print('EndTime:',time.asctime( time.localtime(time.time()) ))
    # destroy the robot node
    
    robot.destroy_node()
    # shutdown previously initialized context
    rclpy.shutdown()
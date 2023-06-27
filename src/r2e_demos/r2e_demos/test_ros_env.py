import rclpy
from ros_environment.scene import RobotClient
from ros_environment.transform import Affine
import time

def main(args=None):
    # initialize ros communications for a given context
    rclpy.init(args=args)

    robot = RobotClient(is_simulation=True)
    time.sleep(5)
    print("StartTime:",time.asctime( time.localtime(time.time()) ))
    robot.close_vacuum_gripper()
    # robot ptp movement again
    print("AfterCloseTime:",time.asctime( time.localtime(time.time()) ))
    
    robot.home()
    #robot.close_vacuum_gripper()
    # back to home pose
    time.sleep(5)
    print("AfterSleep5:",time.asctime( time.localtime(time.time()) ))
    robot.open_vacuum_gripper()
   
    print('EndTime:',time.asctime( time.localtime(time.time()) ))
    # destroy the robot node
    
    robot.destroy_node()
    # shutdown previously initialized context
    rclpy.shutdown()

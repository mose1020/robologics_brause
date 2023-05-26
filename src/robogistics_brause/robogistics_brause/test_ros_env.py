import rclpy
from ros_environment.scene import RobotClient
from ros_environment.transform import Affine
import time

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
    print("StartTime:",time.asctime( time.localtime(time.time()) ))
    
    # # robot ptp movement again
    print("AfterCloseTime:",time.asctime( time.localtime(time.time()) ))
    #-----------pick sth from R5 on the grid----------------------------------------------
    #move to above position
    robot.lin(Affine((0.024, 0.319, 1.121), (0.444, 0.445, 0.550, 0.550)))
    robot.close_vacuum_gripper()
    time.sleep(0.1)

    # # move down
    robot.lin(Affine((0.024, 0.319, 1.021), (0.444, 0.445, 0.550, 0.550)))
    time.sleep(0.1)

    # #move up again and to the drop position
    robot.lin(Affine((0.024, 0.319, 1.121), (0.444, 0.445, 0.550, 0.550)))
    # time.sleep(0.1)
    
    #check in camera
    robot.ptp(Affine((-0.126, 0.270, 1.503), (-0.554, 0.502, -0.436, 0.501)))
    time.sleep(1)
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
    print('EndTime:',time.asctime( time.localtime(time.time()) ))
    # # destroy the robot node
    
    robot.destroy_node()
    # shutdown previously initialized context
    rclpy.shutdown()
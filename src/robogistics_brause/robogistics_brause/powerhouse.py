import rclpy
from ros_environment.scene import RobotClient
from ros_environment.transform import Affine
import time

def main(args=None):
    # initialize ros communications for a given context
    rclpy.init(args=args)
    
    # initialize robot client node; to open or close the gripper of the real 
    #  robot set is_simulation=False
    robot = RobotClient(is_simulation=True)
    robot.home_pose = Affine((0.122, -0.052, 1.426), (0.004, 0.860, 0.007, 0.510))
    
    # move robot to home pose
    robot.home()
    # robot ptp movement to given pose
    #  first tuple represents cartesian coordinates (x, y, z), the second tuple 
    #  represents rotation in quaternions (x, y, z, w)

    #powerhouse
    robot.ptp(Affine((0.136, -0.405, 1.309), (0.000, 0.990, -0.000, 0.142)))
    robot.lin(Affine((0.182, -0.406, 1.153), (0.000, 0.990, -0.001, 0.142)))
    robot.lin(Affine((0.136, -0.105, 1.153), (0.000, 0.990, -0.000, 0.142)))
    robot.lin(Affine((0.136, -0.105, 1.309), (0.000, 0.990, -0.000, 0.142)))
    robot.lin(Affine((0.136, -0.405, 1.309), (0.000, 0.990, -0.000, 0.142)))

    robot.lin(Affine((0.136, -0.255, 1.4), (0.000, 0.990, -0.000, 0.142)))
    robot.lin(Affine((0.136, -0.105, 1.309), (0.000, 0.990, -0.000, 0.142)))

    # open gripper (does nothing when only in simulation)

    # back to home pose
    robot.home()

    # destroy the robot node
    robot.destroy_node()
    # shutdown previously initialized context
    rclpy.shutdown()

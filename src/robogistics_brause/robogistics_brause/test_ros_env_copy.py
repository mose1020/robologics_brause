import rclpy
from ros_environment.scene import RobotClient
from ros_environment.transform import Affine
import time

def main(args=None):
    # initialize ros communications for a given context
    rclpy.init(args=args)
    
    # initialize robot client node; to open or close the gripper of the real 
    #  robot set is_simulation=False
    robot = RobotClient(is_simulation=False)
    robot.home_pose = Affine((0.122, -0.052, 1.426), (0.004, 0.860, 0.007, 0.510))
    
    # move robot to home pose
    robot.home()
    # robot ptp movement to given pose
    #  first tuple represents cartesian coordinates (x, y, z), the second tuple 
    #  represents rotation in quaternions (x, y, z, w)
    robot.ptp(Affine((0.048, -0.308, 1.120), (0.000, 0.946, -0.000, -0.324)))
    # open gripper (does nothing when only in simulation)
    #robot.open_gripper()
    # robot ptp movement again
    robot.ptp(Affine((-0.151, 0.243, 1.111), (0.666, 0.746, 0.014, 0.016)))
    # robot lin movement to given pose
    robot.lin(Affine((-0.051, 0.243, 1.111), (0.666, 0.746, 0.014, 0.016)))
    # close gripper (does nothing when only in simulation)
    #robot.close_gripper()
    # back to home pose
    robot.home()

    # translation of the tcp
    # translation in tcp coordinate system (-0.05 m in z_tcp direction)
    movement_tcp = Affine((0, 0, -0.05))
    # apply translation to current pose (home) given in world coordinates
    pose = robot.home_pose * movement_tcp
    robot.lin(pose)

    # translation in world coordinate system (0.1 in z_world direction)
    movement_world = Affine((0, 0, 0.1))
    # apply translation to current pose given in world coordinates
    pose = movement_world * pose
    robot.lin(pose)

    # destroy the robot node
    robot.destroy_node()
    # shutdown previously initialized context
    rclpy.shutdown()

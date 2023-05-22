import rclpy
from ros_environment.scene import RobotClient
from ros_environment.transform import Affine
import time

def main(args=None):

    # initialize ros communications for a given context
    rclpy.init(args=args)
    # init RobotClient
    robot = RobotClient(is_simulation=False)
    
    # define a homepose
    robot.home_pose = Affine((0.090, -0.051, 1.285), (0.406, 0.511, 0.571, 0.497))
    #move robot to home pose
    robot.home()

    success = robot.lin(Affine((-0.051, 0.243, 1.111), (0.666, 0.746, 0.014, 0.016)))
    print(success)

    robot.home()


    robot.destroy_node()
    # shutdown previously initialized context
    rclpy.shutdown()

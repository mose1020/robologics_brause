import rclpy
from ros_environment.scene import RobotClient
from ros_environment.transform import Affine
import time

from robogistics_brause.tf_tools import TfSubscriber
from robogistics_brause.tf_tools import PoseTracker
from robogistics_brause.tf_tools import transform_list


def main(args=None):

    # initialize ros communications for a given context
    rclpy.init(args=args)

    # init RobotClient
    is_simulation = True # if True: only simulation, if False: real robot
    robot = RobotClient(is_simulation=is_simulation)

    # init PoseTracker # wenn fehler --> schauen ob tf_listener node läuft
    tf_subscriber = TfSubscriber()
    pose = PoseTracker(tf_subscriber)


    current_pose = pose.get_current_pose()
    print(current_pose)

    # home_pose = ((0.122, -0.052, 1.426),(0.004, 0.860, 0.007, 0.510))
    # robot.home_pose = Affine(home_pose)

    robot.home_pose = Affine(((0.177, -0.053, 1.262),(0.424, 0.424, 0.566, 0.566)))


    #robot.home_pose = Affine(transform_list(home_pose))
    robot.home()

    robot.destroy_node()
    # shutdown previously initialized context
    rclpy.shutdown()


if __name__ == "__main__":
    main()
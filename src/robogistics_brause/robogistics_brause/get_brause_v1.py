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
    pose = PoseTracker(tf_subscriber,0.03)

    # start with the homepose
    home_pose = [0.177, -0.053, 1.262,0.424, 0.424, 0.566, 0.566]
    home_pose_trans,home_pose_rot = transform_list(home_pose)
    robot.home_pose = Affine(home_pose_trans,home_pose_rot)
    robot.home()
    pose.goal_pose_reached(home_pose)


    pose_1 = [0.122, -0.052, 1.426,0.004, 0.860, 0.007, 0.510]
    pose_1,pose_2 = transform_list(pose_1)
    robot.ptp(Affine(pose_1,pose_2))
    pose.goal_pose_reached(pose_1)
    

    # home_pose = ((0.122, -0.052, 1.426),(0.004, 0.860, 0.007, 0.510))
    # robot.home_pose = Affine(home_pose)

   


    
    

    robot.destroy_node()
    # shutdown previously initialized context
    rclpy.shutdown()


if __name__ == "__main__":
    main()
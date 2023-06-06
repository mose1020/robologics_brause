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
    is_simulation = False # if True: only simulation, if False: real robot
    robot = RobotClient(is_simulation=is_simulation)

    # init PoseTracker # wenn fehler --> schauen ob tf_listener node läuft
    tf_subscriber = TfSubscriber()
    pose = PoseTracker(tf_subscriber,0.04)

    # start with the homepose
    home_pose = [0.096, -0.073, 1.259,0.505, 0.505, 0.495, 0.495]
    home_pose_trans,home_pose_rot = transform_list(home_pose)

    robot.ptp = Affine(home_pose_trans,home_pose_rot)

    time.sleep(5)
    var = pose.goal_pose_reached(home_pose)
    print(var)
    # pose 1 etc...
    pose_1 = [0.096, -0.073, 1.159,0.505, 0.505, 0.495, 0.495]
    pose_1_trans,pose_1_rot = transform_list(pose_1)

    robot.lin(Affine(pose_1_trans,pose_1_rot))

    time.sleep(5)
    var = pose.goal_pose_reached(pose_1)
    print(var)
    
    # back to the homepose
    time.sleep(5)
    robot.lin = Affine(home_pose_trans,home_pose_rot)


    time.sleep(5)
    var = pose.goal_pose_reached(home_pose)
    print(var)
    # shutdown previously initialized context
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
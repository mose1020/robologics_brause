# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from iras_srvs.srv import Pose as PoseSrv
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion, Point
from std_srvs.srv import Trigger
import copy
from .transform import Affine


# TODO use manipulation_tasks protocol for Robot


class RobotClient:
    """ 
    TODO description
    """

    def __init__(self, node: Node = None, is_simulation: bool = False) -> None:
        """
        TODO docstring

        Returns
        -------
        None.

        """
        if node is None:
            self.node = rclpy.create_node("robot_client")
        else:
            self.node = node
        self.move_cli = self.node.create_client(PoseSrv, "/move_to_pose")
        while not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("move service not available, waiting again...")
        self.node.get_logger().info("move service available")

        self.is_simulation = is_simulation
        if not self.is_simulation:
            self.open_cli = self.node.create_client(Trigger, "/open_vacuum_gripper")
            while not self.open_cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info("opening service not available, waiting again...")
            self.close_cli = self.node.create_client(Trigger, "/close_vacuum_gripper")
            while not self.close_cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info("close service not available, waiting again...")
        # TODO where to get home pose from
        self.home_pose = Affine((-0.002, -0.052, 1.278), (0, 0.86, 0, 0.511))

        self.start_servo_client = self.node.create_client(
            Trigger, "servo_service/start_servo")

        self.stop_servo_client = self.node.create_client(
            Trigger, "servo_service/stop_servo")

        while not self.start_servo_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('start_servo_client not available, waiting again...')

        while not self.stop_servo_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('stop_servo_client not available, waiting again...')

    def home(self):
        """
        TODO docstring

        Returns
        -------
        None.

        """
        self.lin(self.home_pose)
        #self.open_vacuum_gripper()

    def ptp(self, pose: Affine):
        """
        TODO docstring

        Parameters
        ----------
        pose : Affine
            DESCRIPTION.

        Returns
        -------
        None.

        """
        success = self.move(self.affine_to_pose(pose), cart=False)
        if not success:
            self.node.get_logger().info(f"PTP movement to {pose} unsuccessful.")

    def lin(self, pose: Affine):
        """
        TODO docstring

        Parameters
        ----------
        pose : Affine
            DESCRIPTION.

        Returns
        -------
        None.

        """
        success = self.move(self.affine_to_pose(pose), cart=True)
        if not success:
            self.node.get_logger().info(f"LIN movement to {pose} unsuccessful.")

    def open_vacuum_gripper(self) -> bool:
        """
        TODO docstring

        Returns
        -------
        bool
            DESCRIPTION.

        """
        s = True
        if not self.is_simulation:
            future = self.send_open_request()
            response = self.wait_for_response(future)
            s = response.success
        if not s:
            self.node.get_logger().info("Opening gripper unsuccessful.")
        return s

    def close_vacuum_gripper(self) -> bool:
        """
        TODO docstring

        Returns
        -------
        bool
            DESCRIPTION.

        """
        s = True
        if not self.is_simulation:
            future = self.send_close_request()
            response = self.wait_for_response(future)
            s = response.success
        if not s:
            self.node.get_logger().info("Closing gripper unsuccessful.")
        return s

    def send_move_request(self, pose, cart=False) -> Future:
        """
        TODO docstring

        Parameters
        ----------
        pose : TYPE
            DESCRIPTION.
        cart : TYPE, optional
            DESCRIPTION. The default is False.

        Returns
        -------
        Future
            DESCRIPTION.

        """
        req = PoseSrv.Request()
        req.pose = pose
        req.cart = cart
        future = self.move_cli.call_async(req)
        return future

    def send_open_request(self) -> Future:
        """
        TODO docstring

        Returns
        -------
        Future
            DESCRIPTION.

        """
        req = Trigger.Request()
        future = self.open_cli.call_async(req)
        return future

    def send_close_request(self) -> Future:
        """
        TODO docstring

        Returns
        -------
        Future
            DESCRIPTION.

        """
        req = Trigger.Request()
        future = self.close_cli.call_async(req)
        return future

    def wait_for_response(self, future):
        """
        TODO docstring

        Parameters
        ----------
        future : TYPE
            DESCRIPTION.

        Returns
        -------
        response : TYPE
            DESCRIPTION.

        """
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.node.get_logger().info(
                        'Service call failed %r' % (e,))
                    return None
                else:
                    return response

    @staticmethod
    def affine_to_pose(affine: Affine) -> PoseMsg:
        """
        TODO docstring

        Parameters
        ----------
        affine : Affine
            DESCRIPTION.

        Returns
        -------
        PoseMsg
            DESCRIPTION.

        """
        # TODO move to a new file util.py?
        t = affine.translation
        r = affine.quat
        msg = PoseMsg(
            position=Point(x=t[0], y=t[1], z=t[2]),
            orientation=Quaternion(x=r[0], y=r[1], z=r[2], w=r[3]))
        return msg

    @staticmethod
    def pose_to_affine(pose: PoseMsg):
        """
        TODO docstring

        Parameters
        ----------
        pose : PoseMsg
            DESCRIPTION.

        Returns
        -------
        Affine
            DESCRIPTION.
        """
        # TODO move to a new file util.py?
        affine = Affine(
            translation=[pose.position.x, pose.position.y, pose.position.z],
            rotation=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )
        return affine

    def move(self, target_pose: PoseMsg, cart: bool = False) -> bool:
        """
        TODO docstring

        Parameters
        ----------
        target_pose : PoseMsg
            DESCRIPTION.
        cart : bool, optional
            DESCRIPTION. The default is False.

        Returns
        -------
        bool
            DESCRIPTION.

        """
        print("Moving to pose: ", target_pose)
        future = self.send_move_request(target_pose, cart)
        response = self.wait_for_response(future)
        return response.success

    def enable_servo(self):
        trigger = Trigger.Request()
        future = self.start_servo_client.call_async(trigger)
        response = self.wait_for_response(future)
        return response

    def disable_servo(self):
        trigger = Trigger.Request()
        future = self.stop_servo_client.call_async(trigger)
        response = self.wait_for_response(future)
        return response

    def destroy_node(self):
        self.node.destroy_node()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        # create a PoseStamped message and set its values
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map' # frame of reference
        pose_msg.pose.position.x = -0.250 # x coordinate of position
        pose_msg.pose.position.y = -0.484 # y coordinate of position
        pose_msg.pose.position.z = 1.193 # z coordinate of position
        pose_msg.pose.orientation.x = 0.009 # x coordinate of orientation quaternion
        pose_msg.pose.orientation.y = 0.932 # y coordinate of orientation quaternion
        pose_msg.pose.orientation.z = 0.001 # z coordinate of orientation quaternion
        pose_msg.pose.orientation.w = -0.362 # w coordinate of orientation quaternion

        self.publisher_.publish(pose_msg)
        self.get_logger().info('Published pose')

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin_once(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


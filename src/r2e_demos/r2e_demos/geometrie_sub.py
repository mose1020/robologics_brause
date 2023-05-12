
import rclpy
from geometry_msgs.msg import PoseStamped
from ros_environment.scene import RobotClient
from ros_environment.transform import Affine
from rclpy.node import Node


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            'pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):

        global x, y, z, a, b, c, d

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        a = float(msg.pose.orientation.x)
        b = float(msg.pose.orientation.y)
        c = float(msg.pose.orientation.z)
        d = float(msg.pose.orientation.w)

        self.get_logger().info(f"Position: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}) \nOrientation: ({msg.pose.orientation.x}, {msg.pose.orientation.y}, {msg.pose.orientation.z}, {msg.pose.orientation.w})")

        robot = RobotClient(is_simulation=True)

        robot.ptp(Affine((x,y,z), (a, b, c, d)))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
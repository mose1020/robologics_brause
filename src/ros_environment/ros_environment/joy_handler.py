import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped


class JoyHandler(Node):
    def __init__(self):
        super(JoyHandler, self).__init__('joy_handler')

        self.twist_publisher = self.create_publisher(TwistStamped, '/servo_service/delta_twist_cmds', 10)

        self.linear_velocity_scaling = 0.2
        self.angular_velocity_scaling = 0.4

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 1)

    def joy_callback(self, data):

        twist = TwistStamped()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = -data.axes[3] * self.linear_velocity_scaling
        twist.twist.linear.y = data.axes[4] * self.linear_velocity_scaling
        twist.twist.linear.z = data.axes[1] * self.linear_velocity_scaling

        right_rot = 1 - (data.axes[5] + 1) / 2
        left_rot = 1 - (data.axes[2] + 1) / 2

        rot = 0.0

        if right_rot > 0.05:
            rot = -right_rot * self.angular_velocity_scaling
        elif left_rot > 0.05:
            rot = left_rot * self.angular_velocity_scaling

        twist.twist.angular.z = rot

        now = self.get_clock().now().to_msg()
        twist.header.stamp = now
        self.twist_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    robot_client = JoyHandler()
    rclpy.spin(robot_client)
    robot_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

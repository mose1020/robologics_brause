import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException
from geometry_msgs.msg import TransformStamped

class TfListener(Node):

    def __init__(self, first_link, second_link):
        super().__init__('tf_listener')
        self.first_name_ = first_link
        self.second_name_ = second_link
        self.get_logger().info("Transforming from {} to {}".format(self.second_name_, self.first_name_))
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.publisher_ = self.create_publisher(TransformStamped, "tf_topic/TransformStamped",10) # change the path
        self.timer = self.create_timer(0.33, self.timer_callback) #30 Hz = 0.333s

    def timer_callback(self):
        try:
            trans = self._tf_buffer.lookup_transform(self.second_name_, self.first_name_, rclpy.time.Time())
            self.publisher_.publish(trans)

        except LookupException as e:
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))


def main():
    rclpy.init()
    node = TfListener('cell_link', 'realsense_camera')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
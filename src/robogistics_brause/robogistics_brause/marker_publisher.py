import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.declare_parameter('marker_position', [0.0, 0.0, 1.2])  # Declare parameter with default position
        self.get_logger().info('Marker publisher node initialized')

    def publish_marker(self,position):
        marker_msg = Marker()
        marker_msg.header.frame_id = 'base_link'  # Set the frame ID for the marker
        marker_msg.type = Marker.SPHERE  # Set the marker type to a sphere (can be changed based on your requirements)
        
        # Retrieve marker position from the parameter
        #position = self.get_parameter('marker_position').value
        marker_msg.pose.position = Point(x=position[0], y=position[1], z=position[2])  # Set the X, Y, Z coordinates
        
        marker_msg.scale.x = 0.02  # Set the scale of the marker
        marker_msg.scale.y = 0.02
        marker_msg.scale.z = 0.001
        marker_msg.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Set the color (red in this example)
        
        self.publisher_.publish(marker_msg)
        self.get_logger().info('Marker published')

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()
    rclpy.spin(marker_publisher)
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

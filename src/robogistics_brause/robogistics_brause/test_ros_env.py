import rclpy
from rclpy.node import Node
from ros_environment.scene import RobotClient
from ros_environment.transform import Affine
import time
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import queue
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import Point

from .coordinates_in_camera_frame_leon import getPose
#from .coordinates_in_camera_frame import getPose


class BrausePicker:
    def __init__(
            self,
            is_simulation: bool = True,
            home_pose: Affine = Affine((0.006, -0.053, 1.216), (0.608, 0.608, 0.361, 0.361)),
            ) -> None:
        """ Initialize the cubes demo class.

        Parameters
        ----------
        is_simulation : bool, default=False
            Defines if the demo runs on the real robot or in simulation.
        home_pose : Affine, default=Affine((0.122, -0.052, 1.426), (0.004, 0.860, 0.007, 0.510)
            The robot's cartesian home pose.
        

        """
        self.robot = RobotClient(is_simulation=is_simulation)
        self.robot.home_pose=home_pose
    
    def get_pre_pose(self, pose: Affine, distance: float = 0.1) -> Affine:
        """ Calculate the pre-pick and pre-place pose with the given distance 
        to the given pose.

        Parameters
        ----------
        pose : Affine
            The final pick/place pose.
        distance : float, default=0.05
            Distance between pre-pose and final pose.

        Returns
        -------
        pre_pose : TYPE
            The pre-pose (pre-pick/pre-place pose).

        """
        #pre_pose_transform = Affine(translation=(0, 0, -distance))
        #pre_pose = pose * pre_pose_transform
        
        # prepose is 10cm above the pick pose
        pre_pose = Affine((pose.translation[0], pose.translation[1], pose.translation[2] + distance), pose.quat)

        return pre_pose
     
    def pick(self, pick_pose: Affine) -> None:
        """ Pick process: activate the vacuumgripper, move to pre-pick pose (ptp),
        move to pick pose (lin), move back to pre-pick pose 
        (lin).
        
        Parameters
        ----------
        pick_pose : Affine
            The pick pose.

        """
        self.robot.home()
        pre_pick = self.get_pre_pose(pick_pose, distance=0.1)
        self.robot.ptp(pre_pick)
        self.robot.close_vacuum_gripper()
        self.robot.lin(pick_pose)
        #maybe a wait is needed here
        self.robot.lin(pre_pick) 
        self.robot.home()

    def move_to_camera(self) -> None:
        self.robot.home()
        #move to camera position via home to avoid collision and entanglement
        self.robot.ptp(Affine((0.051, 0.182, 1.319), (0.732, 0.451, 0.434, 0.268)))
        self.robot.ptp(Affine((0.028, 0.293, 1.277), (0.848, 0.140, 0.504, 0.084)))
        self.robot.ptp(Affine((-0.101, 0.261, 1.507), (-0.724, -0.063, 0.025, 0.687)))

    def leave_camera(self) -> None:
        self.robot.ptp(Affine((0.028, 0.293, 1.277), (0.848, 0.140, 0.504, 0.084)))
        self.robot.ptp(Affine((0.051, 0.182, 1.319), (0.732, 0.451, 0.434, 0.268)))
        self.robot.home()


    def drop_at_slide(self) -> None:
        #drop position
        self.robot.home()
        self.robot.ptp(Affine((0.109, -0.389, 1.221), (0.497, 0.497, 0.503, 0.503)))
        self.robot.open_vacuum_gripper()
        time.sleep(2.0)
        self.robot.home()

    def home(self) -> None:
        """ Move the robot to its home pose.

        """
        self.robot.home()

    def shutdown(self) -> None:
        """ Finish up the demo by destroying the RobotClient node.
            
        """
        self.robot.destroy_node()

class MarkerPublisher(Node): # Wenn der Marker verschwindet --> vielleicht is der dann in der Roboterbox (wenn tiefbild zu ungenau)
                                                            # --> scale.z von marker in dem fall erhöhen
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.declare_parameter('marker_position', [0.0, 0.0, 1.03])  # Declare parameter with default position
        self.get_logger().info('Marker publisher node initialized')

    def publish_marker(self,position):
        marker_msg = Marker()
        marker_msg.header.frame_id = 'cell_link'  # Set the frame ID for the marker
        marker_msg.type = Marker.SPHERE  # Set the marker type to a sphere (can be changed based on your requirements)
        
        marker_msg.pose.position = Point(x=position[0], y=position[1], z=position[2])  # Set the X, Y, Z coordinates
        
        marker_msg.scale.x = 0.05  # Set the scale of the marker
        marker_msg.scale.y = 0.05
        marker_msg.scale.z = 0.005
        marker_msg.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)  # Set the color (red in this example)
        
        self.publisher_.publish(marker_msg)
        self.get_logger().info('Marker published')

class MarkerPublisherCam(Node): # Wenn der Marker verschwindet --> vielleicht is der dann in der Roboterbox (wenn tiefbild zu ungenau)
                                                            # --> scale.z von marker in dem fall erhöhen
    def __init__(self):
        super().__init__('marker_publisher_cam')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker_cam', 10)
        self.declare_parameter('marker_position', [0.0, 0.0, 1.03])  # Declare parameter with default position
        self.get_logger().info('Marker publisher node initialized')

    def publish_marker(self,position):
        marker_msg = Marker()
        marker_msg.header.frame_id = 'realsense_camera_origin'  # Set the frame ID for the marker
        marker_msg.type = Marker.SPHERE  # Set the marker type to a sphere (can be changed based on your requirements)
        
        marker_msg.pose.position = Point(x=position[0], y=position[1], z=position[2])  # Set the X, Y, Z coordinates
        
        marker_msg.scale.x = 0.05  # Set the scale of the marker
        marker_msg.scale.y = 0.05
        marker_msg.scale.z = 0.005
        marker_msg.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Set the color (red in this example)
        
        self.publisher_.publish(marker_msg)
        self.get_logger().info('Marker published')

class TfSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')
        self.subscriber = self.create_subscription(TransformStamped, 'tf_topic/TransformStamped', self.listener_callback, 10)
        self.msg_queue = queue.Queue()

    def listener_callback(self, msg):
        self.msg_queue.put(msg)

class TfTransformer:

    def __init__(self,tf_subscriber):
        
        self.tf_subscriber = tf_subscriber

    def get_transformation(self):

        while self.tf_subscriber.msg_queue.empty():
            rclpy.spin_once(self.tf_subscriber)
            time.sleep(0.01)  # Optional delay to avoid busy waiting
        # Retrieve the message from the queue
        msg = self.tf_subscriber.msg_queue.get()

        return msg

    def make_transformation(self, source_pose):

        transform = self.get_transformation()

        # Create a PoseStamped message in the source frame
        source_pose_ = geometry_msgs.msg.Pose()
        source_pose_.position.x = source_pose[0]
        source_pose_.position.y = source_pose[1]
        source_pose_.position.z = source_pose[2]

        transformed_pose = tf2_geometry_msgs.do_transform_pose(source_pose_, transform)
 
        return transformed_pose


def main(args=None):

    # initialize ros communications for a given context
    rclpy.init(args=args)

    marker_camara_pose = MarkerPublisherCam()
    marker_transformed_pose = MarkerPublisher()
    test_picks = BrausePicker(is_simulation=False)
    #tf_subscriber = TfSubscriber()
    #tf_transformer = TfTransformer(tf_subscriber)

    tf_fix = [-0.115,-0.260,1.661]
    # user chooses color and if its available the robot picks it otherwise new color is chosen
    position_from_camera = getPose()
    print("POSE ", position_from_camera)
    #transformed_position = tf_transformer.make_transformation(position_from_camera)
    transformed_position2 = [position_from_camera[0]-tf_fix[0],position_from_camera[1]-tf_fix[1],position_from_camera[2]-tf_fix[2]]

    #print("Transformed_POSE ", transformed_position)
    print("Transformed_POSE 2", transformed_position2)
    marker_camara_pose.publish_marker([position_from_camera[0],position_from_camera[1],position_from_camera[2]-0.02]) # z immer etwas weniger, dass man den marker sieht
    #marker_transformed_pose.publish_marker([transformed_position.position.x,transformed_position.position.y,1.03]) 
    marker_transformed_pose.publish_marker([transformed_position2[0],transformed_position2[1],1.03]) 
    time.sleep(5) # wichtig, das die marker geändert werden
    
    #pose_from_camera = Affine((-0.070, 0.327, 1.03), (0.444, 0.445, 0.550, 0.550)) # Fixposition von Stephe

    pose_from_camera = Affine((transformed_position2[0],transformed_position2[1],1.02), (0.444, 0.445, 0.550, 0.550))
  
    
    test_picks.pick(pose_from_camera)


    #test_picks.move_to_camera()
    # ###### bildmethode check
    #test_picks.leave_camera()
    # # wenn erfolgreich

    test_picks.drop_at_slide()

    
    #test_picks.shutdown()
    # shutdown previously initialized context
    #   translation:
    # x: -0.06800000000034354
    # y: -0.26
    # z: 1.6609999999999858



    rclpy.shutdown()

if __name__ == '__main__':
    main()




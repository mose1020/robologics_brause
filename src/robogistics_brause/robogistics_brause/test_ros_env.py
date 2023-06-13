import rclpy
from rclpy.node import Node
from ros_environment.scene import RobotClient
from ros_environment.transform import Affine
import time
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA




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
        #time.sleep(1.0)
        # print("pre_pick")
        # print(pre_pick)

        # print("pick_pose")
        # print(pick_pose)

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

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.declare_parameter('marker_position', [0.0, 0.0, 1.2])  # Declare parameter with default position
        self.get_logger().info('Marker publisher node initialized')

    def publish_marker(self,position):
        marker_msg = Marker()
        marker_msg.header.frame_id = 'cell_link'  # Set the frame ID for the marker
        marker_msg.type = Marker.SPHERE  # Set the marker type to a sphere (can be changed based on your requirements)
        
        # Retrieve marker position from the parameter
        #position = self.get_parameter('marker_position').value
        marker_msg.pose.position = Point(x=position[0], y=position[1], z=position[2])  # Set the X, Y, Z coordinates
        
        marker_msg.scale.x = 0.2  # Set the scale of the marker
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.001
        marker_msg.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Set the color (red in this example)
        
        self.publisher_.publish(marker_msg)
        self.get_logger().info('Marker published')

def main(args=None):

    # initialize ros communications for a given context
    rclpy.init(args=args)

    marker = MarkerPublisher()

    # create a new demo instance
    test_picks = BrausePicker(is_simulation=False)

    # user chooses color and if its available the robot picks it otherwise new color is chosen
    pose_from_camera = Affine((-0.070, 0.327, 1.03), (0.444, 0.445, 0.550, 0.550))
    print(pose_from_camera.translation)
    marker.publish_marker(pose_from_camera.translation)
    time.sleep(15) # fürs video
    test_picks.pick(pose_from_camera)
    # test_picks.move_to_camera()
    ###### bildmethode check
    # test_picks.leave_camera()
    marker.publish_marker([0.109, -0.389, 1.22])

    # wenn erfolgreich
    test_picks.drop_at_slide()

    
    test_picks.shutdown()
    # shutdown previously initialized context
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# - Translation: [0.051, 0.182, 1.319]
# - Rotation: in Quaternion [0.732, 0.451, 0.434, 0.268]

# - Translation: [0.028, 0.293, 1.277]
# - Rotation: in Quaternion [0.848, 0.140, 0.504, 0.084]

# - Translation: [-0.101, 0.261, 1.507]
# - Rotation: in Quaternion [-0.724, -0.063, 0.025, 0.687]



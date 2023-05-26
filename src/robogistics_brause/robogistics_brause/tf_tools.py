#from tf2_msgs.msg import TFMessage
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import queue
import time
import numpy as np


class TfSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')
        self.subscriber = self.create_subscription(TransformStamped, 'tf_topic/TransformStamped', self.listener_callback, 10)
        self.msg_queue = queue.Queue()

    def listener_callback(self, msg):
        self.msg_queue.put(msg)


class PoseTracker:

    def __init__(self,tf_subscriber,tolerance):
        
        self.tf_subscriber = tf_subscriber
        self.tolerance = tolerance

    def get_current_pose(self):

        while self.tf_subscriber.msg_queue.empty():
            rclpy.spin_once(self.tf_subscriber)
            time.sleep(0.01)  # Optional delay to avoid busy waiting
        # Retrieve the message from the queue
        msg = self.tf_subscriber.msg_queue.get()
        translation_x = msg.transform.translation.x
        translation_y = msg.transform.translation.y
        translation_z = msg.transform.translation.z

        rotation_x = msg.transform.rotation.x
        rotation_y = msg.transform.rotation.y
        rotation_z = msg.transform.rotation.z
        rotation_w = msg.transform.rotation.w
        
        return [translation_x,translation_y,translation_z,rotation_x,rotation_y,rotation_z,rotation_w]
    

    def get_pose_dif(self,goal_pose):

        current_pose = self.get_current_pose()
        current_pose = np.array(current_pose)
        goal_pose = np.array(goal_pose)
        pose_dif = np.subtract(goal_pose,current_pose)

        return pose_dif
    
    def goal_pose_reached(self,goal_pose): 

        pose_dif = self.get_pose_dif(goal_pose)
        current_tolerance = np.mean(pose_dif[:3])

        while current_tolerance > self.tolerance:
            pose_dif = self.get_pose_dif(goal_pose)
            current_tolerance = np.mean(pose_dif[:3])
            time.sleep(0.1)
            print("Current Tolerance to Homepose: ",current_tolerance)
            continue

        return True



def transform_list(pose):
    first_tuple = tuple(pose[:3])
    second_tuple = tuple(pose[3:])
    return first_tuple, second_tuple

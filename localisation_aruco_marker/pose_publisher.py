#!/usr/bin/env python3

'''
-------------------------- LOCALISATION ARUCO MARKER SCRIPT --------------------------
This python script initialises and runs ‘MarkerTrackingNode’, a ROS2 node that
  detects an ARUCO marker in a camera feed and estimates its pose relative to the
  camera. It then publishes this pose information to ROS2 topics and transforms it 
  into a global reference frame.

  
REV 0 ....................................................................... 06/03/25
- This is the version made by Alexander Allan, edited by Clayder Gonzalez for
  integration with ROS2 environment.

'''
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy # create ROS2 node
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_matrix
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# import tf2_ros
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
# from tf2_ros.transformations import quaternion_from_matrix
from geometry_msgs.msg import TransformStamped

class PosePublisher(Node):
    def __init__(self):
        super().__init__('node_marker_tracking')
        self.marker_pose_pub = self.create_publisher(PoseStamped, '/rover_centre', 10)
        self.marker_pose_pub2 = self.create_publisher(PoseStamped, '/camera2base_link', 10)

        self.tf_buffer = Buffer()
        self.br = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1/100, self.main_loop)

    def main_loop(self):

        # Publish Pose
        try:
            #transform_dot = self.tf_buffer.lookup_transform('camera', 'base_link', rclpy.time.Time())
            #transform = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time())

            transform_dot = self.tf_buffer.lookup_transform('camera', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            transform = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation = transform.transform.rotation
            self.marker_pose_pub.publish(pose_msg)

            #print(pose_msg.pose.position.x, ',', pose_msg.pose.position.y, ',', pose_msg.pose.position.z)
                
            pose_msg2 = PoseStamped()
            pose_msg2.header.stamp = self.get_clock().now().to_msg()
            pose_msg2.header.frame_id = "camera"
            pose_msg2.pose.position.x = transform_dot.transform.translation.x
            pose_msg2.pose.position.y = transform_dot.transform.translation.y
            pose_msg2.pose.position.z = transform_dot.transform.translation.z
            pose_msg2.pose.orientation = transform_dot.transform.rotation
            self.marker_pose_pub2.publish(pose_msg2)

        except Exception as e:
            self.get_logger().warn(f"Transform error: {e}")
        
        
def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


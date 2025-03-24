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

# Parameters
camera_matrix = np.array([
    [1375.5, 0, 977.4],
    [0, 1376.7, 555.5],
    [0, 0, 1]
])
dist_coeffs = np.array([0.1196, -0.2006, 0, 0, 0])
marker_sizes = {5: 0.078}

class MarkerTrackingNode(Node):
    def __init__(self):
        super().__init__('node_marker_tracking')
        self.video_capture = None
        self.rover_position = None
        self.bridge = CvBridge()

        # GET IMAGE
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        #self.marker_pose_pub = self.create_publisher(PoseStamped, '/rover_centre', 10)
        #self.marker_pose_pub2 = self.create_publisher(PoseStamped, '/camera2base_link', 10)

        # SETUP BROADCASTING FOR MARKERTRACKINGNODE
        #self.tf_buffer = Buffer()
        self.br = TransformBroadcaster(self)
        #self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1/100, self.main_loop)

    # CONVERT IMAGE MESSAGE TO CV2
    def image_callback(self, data):
        try:
            self.video_capture = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Image conversion error: {e}")

    # ESTIMATE ROVER POSE - ARUCOMARKER2CAMERA
    def estimate_pose(self, corners, marker_size):
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                  [marker_size / 2, marker_size / 2, 0],
                                  [marker_size / 2, -marker_size / 2, 0],
                                  [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        rvecs, tvecs = [], []
        for c in corners:
            _, R, t = cv2.solvePnP(marker_points, c, camera_matrix, dist_coeffs, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
        return rvecs, tvecs


    def main_loop(self):
        # CHECK IF VIDEO CAPTURE
        if self.video_capture is None:
            return

        # GET CAPTURE AND CONVERT TO CV2 IMAGE
        frame = self.video_capture
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)

        # IF ARUCO MARKER DETECTED
        if ids is not None:
            detected_ids = ids.flatten()
            if 5 in detected_ids:

                # DETECT ID 5 AND ESTIMATE ITS POSE
                i = np.where(detected_ids == 5)[0][0]
                marker_size = marker_sizes[5]
                rvecs, tvecs = self.estimate_pose(corners[i], marker_size)

                # Transformation matrix and quaternion
                R, _ = cv2.Rodrigues(rvecs[0])
                R = np.vstack([np.hstack([R, np.array([[0], [0], [0]])]), [0, 0, 0, 1]])
                q = quaternion_from_matrix(R)
                x, y, z = tvecs[0].flatten()
                
                # Publish Transform
                # ARUCOMARKER2CAMERA IN TF2 FORMAT
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "camera"
                t.child_frame_id = "marker_frame"
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = z
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                self.br.sendTransform(t)

              # ... PUBLISHES 't' TO MarkerTrackingNode().

                #print('hello')

                '''
                # Publish Pose
                try:
                    #transform_dot = self.tf_buffer.lookup_transform('camera', 'base_link', rclpy.time.Time()) # no timeout; waits until a valid transform is available or lookup fails
                    #transform = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time())

                    transform_dot = self.tf_buffer.lookup_transform('camera', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.0)) # function is non-blockin; immediately returns if no valid transform is available at the requested time
                    transform = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.0))

                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "world"
                    pose_msg.pose.position.x = transform.transform.translation.x
                    pose_msg.pose.position.y = transform.transform.translation.y
                    pose_msg.pose.position.z = transform.transform.translation.z
                    pose_msg.pose.orientation = transform.transform.rotation
                    self.marker_pose_pub.publish(pose_msg)

                    print(pose_msg.pose.position.x, ',', pose_msg.pose.position.y, ',', pose_msg.pose.position.z)
                     
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
                '''
        
        cv2.imshow('Frame with Pose Estimation', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MarkerTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


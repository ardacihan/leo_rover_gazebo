import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations

R_OPT_TO_CAM = np.array([[ 0,  0,  1],
                           [-1,  0,  0],
                           [ 0, -1,  0]], dtype=float)


class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')

        self.declare_parameter('robot_ns', 'leo1')
        self.robot_ns = self.get_parameter('robot_ns').value

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame_id = f'{self.robot_ns}/sensor_camera_link'
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()
        self.last_ids = None

        if not hasattr(cv2, "aruco"):
            self.get_logger().error("cv2.aruco not available.")
            raise RuntimeError("Missing cv2.aruco")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()

        self.create_subscription(CameraInfo, f'/{self.robot_ns}/camera/camera_info', self.camera_info_cb, 10)
        self.create_subscription(Image, f'/{self.robot_ns}/camera/image', self.image_callback, 10)

        self.status_pub = self.create_publisher(String, 'aruco/status', 10)
        self.create_timer(0.5, self.publish_status)

        self.get_logger().info(f"Aruco detector started for {self.robot_ns}")

    def camera_info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.camera_frame_id = msg.header.frame_id

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        corners, ids, _ = cv2.aruco.detectMarkers(
            image=frame,
            dictionary=self.aruco_dict,
            parameters=self.params
        )

        if ids is not None and len(ids) > 0 and self.camera_matrix is not None:
            self.last_ids = ids.flatten().tolist()
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, 0.2, self.camera_matrix, self.dist_coeffs
            )
            for j, marker_id in enumerate(ids.flatten()):
                # Convert tvec from optical to ROS camera_link frame
                tvec_ros = R_OPT_TO_CAM @ tvecs[j][0]

                # Convert rvec to rotation matrix, then to ROS frame
                R_marker_opt, _ = cv2.Rodrigues(rvecs[j])
                R_marker_ros = R_OPT_TO_CAM @ R_marker_opt

                # Build 4x4 transform matrix
                m = np.eye(4)
                m[:3, :3] = R_marker_ros
                q = tf_transformations.quaternion_from_matrix(m)

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = self.camera_frame_id
                t.child_frame_id = f'aruco_{marker_id}'
                t.transform.translation.x = float(tvec_ros[0])
                t.transform.translation.y = float(tvec_ros[1])
                t.transform.translation.z = float(tvec_ros[2])
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                self.tf_broadcaster.sendTransform(t)
        else:
            self.last_ids = None

    def publish_status(self):
        msg = String()
        msg.data = f"Detected ArUco IDs: {self.last_ids}" if self.last_ids else "No ArUco detected"
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = ArucoDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
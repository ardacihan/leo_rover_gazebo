import rclpy, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

rclpy.init()
n = Node('debug_color_throttle')
pub = n.create_publisher(Image, '/debug/color_5hz', 5)
pub_i = n.create_publisher(CameraInfo, '/debug/color_5hz/camera_info', 5)
last = [0.0]
info = [None]
def cb(m):
    t = time.monotonic()
    if t - last[0] >= 0.2:
        last[0] = t
        pub.publish(m)
        if info[0] is not None:
            pub_i.publish(info[0])
def icb(m):
    info[0] = m
n.create_subscription(Image, '/camera/camera/color/image_raw', cb, qos_profile_sensor_data)
n.create_subscription(CameraInfo, '/camera/camera/color/camera_info', icb, qos_profile_sensor_data)
rclpy.spin(n)

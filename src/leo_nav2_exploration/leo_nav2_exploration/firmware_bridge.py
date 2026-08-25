#!/usr/bin/env python3
"""Copy the few firmware topics between the rover domain and a localhost Nav2 domain.

Nav2 costmaps on the rover's ROS domain multicast ~1.4 MB/s over Ethernet and
the firmware node drops /cmd_vel. Mapping therefore runs on another domain
with ROS_LOCALHOST_ONLY=1. This process is two children: one talks to the
rover, one talks to Nav2, and they share only odom, battery, and cmd_vel.

Do not import rclpy at module level. Each child sets its domain, then imports.

Defaults are jetson-02 (domain 2, root topics). Rover 4 must set
LEO_FIRMWARE_DOMAIN=4 and the /rob_2 topic env vars on that robot -- never
leave domain 4 as the default or a jetson-02 bridge will publish /cmd_vel
onto rover 4.
"""

import multiprocessing as mp
import os
import time
from queue import Empty, Full


def firmware_bridge_settings():
    """Return domain/topic settings. Defaults are jetson-02 root topics."""
    return {
        "firmware_domain": os.environ.get("LEO_FIRMWARE_DOMAIN", "2"),
        "nav_domain": os.environ.get("LEO_NAV_DOMAIN", "22"),
        "odom_topic": os.environ.get("LEO_FW_ODOM_TOPIC", "/wheel_odom"),
        "battery_topic": os.environ.get(
            "LEO_FW_BATTERY_TOPIC", "/firmware/battery_averaged"
        ),
        "cmd_topic": os.environ.get("LEO_FW_CMD_VEL_TOPIC", "/cmd_vel"),
    }


def put_latest(queue, item):
    """Keep only the newest sample so a slow side cannot back up."""
    while True:
        try:
            queue.put_nowait(item)
            return
        except Full:
            try:
                queue.get_nowait()
            except Empty:
                pass


def _firmware_side(odom_q, batt_q, cmd_q, settings):
    """Domain of the rover SBC: subscribe sensors, publish /cmd_vel."""
    os.environ["ROS_DOMAIN_ID"] = settings["firmware_domain"]
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    # Must run before rclpy.init(): builtin FastDDS transports create SHM
    # segments that wedge the firmware endpoint when they accumulate.
    from leo_nav2_exploration.fastdds_profiles import apply_udp_only_transport
    apply_udp_only_transport()
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float32

    rclpy.init()
    node = rclpy.create_node("firmware_bridge_fw")
    cmd_pub = node.create_publisher(Twist, settings["cmd_topic"], 10)
    last_cmd = [0.0]
    node.create_subscription(
        Odometry, settings["odom_topic"], lambda m: put_latest(odom_q, m),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Float32, settings["battery_topic"],
        lambda m: put_latest(batt_q, float(m.data)),
        qos_profile_sensor_data,
    )

    def drain_cmd():
        now = time.monotonic()
        try:
            cmd_pub.publish(cmd_q.get_nowait())
            last_cmd[0] = now
            return
        except Empty:
            pass
        # Fail closed: if Nav2 stops talking, do not hold the last non-zero.
        if now - last_cmd[0] > 0.4:
            cmd_pub.publish(Twist())
            last_cmd[0] = now

    timer = node.create_timer(0.02, drain_cmd)
    node.get_logger().info(
        f"firmware side on domain {settings['firmware_domain']}: "
        f"{settings['odom_topic']} / {settings['battery_topic']} in, "
        f"{settings['cmd_topic']} out"
    )
    try:
        rclpy.spin(node)
    finally:
        timer.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _nav_side(odom_q, batt_q, cmd_q, settings):
    """Localhost Nav2 domain: publish sensors + TF, subscribe /cmd_vel."""
    os.environ["ROS_DOMAIN_ID"] = settings["nav_domain"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float32
    from tf2_ros import TransformBroadcaster

    rclpy.init()
    node = rclpy.create_node("firmware_bridge_nav")
    odom_pub = node.create_publisher(Odometry, "/wheel_odom", qos_profile_sensor_data)
    batt_pub = node.create_publisher(
        Float32, "/firmware/battery_averaged", qos_profile_sensor_data
    )
    tf_pub = TransformBroadcaster(node)
    node.create_subscription(
        Twist, "/cmd_vel", lambda m: put_latest(cmd_q, m), 10
    )

    def drain_sensors():
        try:
            odom = odom_q.get_nowait()
        except Empty:
            odom = None
        if odom is not None:
            odom_pub.publish(odom)
            # Stamp with the Jetson clock. Firmware stamps are a few ms
            # behind and Nav2 then fails with "extrapolation into the future".
            tf_msg = TransformStamped()
            tf_msg.header.stamp = node.get_clock().now().to_msg()
            tf_msg.header.frame_id = odom.header.frame_id or "odom"
            tf_msg.child_frame_id = odom.child_frame_id or "base_footprint"
            tf_msg.transform.translation.x = odom.pose.pose.position.x
            tf_msg.transform.translation.y = odom.pose.pose.position.y
            tf_msg.transform.translation.z = odom.pose.pose.position.z
            tf_msg.transform.rotation = odom.pose.pose.orientation
            tf_pub.sendTransform(tf_msg)
        try:
            volts = batt_q.get_nowait()
        except Empty:
            return
        msg = Float32()
        msg.data = volts
        batt_pub.publish(msg)

    timer = node.create_timer(0.02, drain_sensors)
    node.get_logger().info(
        f"nav side on domain {settings['nav_domain']} localhost: odom/battery/TF out, cmd_vel in"
    )
    try:
        rclpy.spin(node)
    finally:
        timer.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    mp.set_start_method("spawn", force=True)
    settings = firmware_bridge_settings()
    odom_q = mp.Queue(maxsize=1)
    batt_q = mp.Queue(maxsize=1)
    cmd_q = mp.Queue(maxsize=1)
    fw_proc = mp.Process(
        target=_firmware_side,
        args=(odom_q, batt_q, cmd_q, settings),
        name="fw",
        daemon=True,
    )
    nav_proc = mp.Process(
        target=_nav_side,
        args=(odom_q, batt_q, cmd_q, settings),
        name="nav",
        daemon=True,
    )
    fw_proc.start()
    nav_proc.start()
    fw_proc.join()
    nav_proc.join()


if __name__ == "__main__":
    main()

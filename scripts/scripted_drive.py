#!/usr/bin/env python3
"""Drive a fixed waypoint route using ground-truth pose.

For a SLAM parameter comparison the *motion* has to be identical in every run.
Driving with Nav2 or a frontier explorer does the opposite: a bad SLAM config
produces a bad map, which produces different paths, which changes what the
lidar ever sees -- so the map score stops measuring SLAM alone.

This driver closes its control loop on Gazebo's ground-truth odometry
(``/leo1/odom``) and ignores the SLAM map entirely, so every configuration is
scored on the same trajectory.

Routes are named sets of (x, y) waypoints in world coordinates. The rover
turns in place to face the next waypoint, then drives to it -- the in-place
turns are deliberate, because that is where a skid-steer's wheel odometry
loses the most yaw.

Usage (inside the sim container):

    python3 /ros2_ws/scripts/scripted_drive.py --ros-args \
        -p route:=office_full -p linear_speed:=0.3 -p angular_speed:=0.5
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Office world (24 x 16 m): a central corridor between y = -1.2 and y = +1.2
# with doorways at x ~ -8, 0, +8 (north wall) and x ~ -6, +6 (south wall).
# Partitions at x = -4 and x = +4 split the north half into three rooms; a
# partition at x = 0 splits the south half into two. The route visits all five
# rooms and returns to the origin, so the run ends on a large loop closure.
#
# The doorway x positions are fixed by the world, so room entries must use them.
# Inside the rooms the legs are offset away from the furniture -- the rover is
# 0.44 m wide and has no obstacle avoidance here, and a leg that merely misses a
# desk's collision box by 0.1 m will wedge against it:
#     desk_n3  x [ 8.1,  8.9]  y [ 3.7,  5.3]
#     desk_n1  x [-9.9, -8.1]  y [ 4.5,  5.5]
#     desk_s1  x [-7.9, -6.1]  y [-5.6, -4.4]
#     pillar_s2  centre (7, -4.5), r 0.4
ROUTES = {
    'office_full': [
        # east along the corridor
        (4.0, 0.0), (8.0, 0.0),
        # north-east room (door at x = 8); skirt desk_n3 on its west side
        (8.0, 2.5), (6.0, 2.5), (6.0, 7.0), (10.8, 7.0), (10.8, 2.5),
        (8.0, 2.5),
        # back to the corridor, out to the east end
        (8.0, 0.0), (10.8, 0.0), (6.0, 0.0),
        # south-east room (door at x = 6); pillar_s2 sits east of the x = 6 leg
        (6.0, -3.0), (6.0, -7.0), (2.0, -7.0), (10.8, -7.0), (10.8, -3.0),
        (6.0, -3.0), (6.0, 0.0),
        # north-centre room (door at x = 0), no furniture
        (0.0, 0.0), (0.0, 3.0), (0.0, 6.8), (-3.0, 6.8), (3.0, 6.8),
        (0.0, 3.0), (0.0, 0.0),
        # south-west room (door at x = -6); desk_s1 sits west of the x = -5 leg
        (-6.0, 0.0), (-6.0, -3.0), (-5.0, -3.0), (-5.0, -7.0),
        (-10.8, -7.0), (-10.8, -3.0), (-5.0, -3.0), (-6.0, -3.0), (-6.0, 0.0),
        # north-west room (door at x = -8); desk_n1 sits west of the x = -6 leg
        (-8.0, 0.0), (-8.0, 3.0), (-6.0, 3.0), (-6.0, 7.0), (-10.8, 7.0),
        (-10.8, 3.0), (-8.0, 3.0), (-8.0, 0.0),
        # west end, then all the way home -- the loop closure
        (-10.8, 0.0), (-6.0, 0.0), (0.0, 0.0),
    ],
    # Short route for smoke tests.
    'office_short': [
        (4.0, 0.0), (8.0, 0.0), (8.0, 2.5), (6.0, 2.5), (8.0, 0.0), (0.0, 0.0),
    ],
}


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ScriptedDrive(Node):

    def __init__(self):
        super().__init__('scripted_drive')
        self.declare_parameter('route', 'office_full')
        self.declare_parameter('odom_topic', '/leo1/odom')
        self.declare_parameter('cmd_topic', '/leo1/cmd_vel')
        self.declare_parameter('linear_speed', 0.30)
        self.declare_parameter('angular_speed', 0.50)
        self.declare_parameter('position_tolerance', 0.20)
        self.declare_parameter('heading_tolerance', 0.05)
        self.declare_parameter('settle_time', 0.4)
        # There is no obstacle avoidance here: if a leg is blocked the rover
        # would push against the obstacle until the run's wall-clock cap. Give
        # up on a waypoint that has stopped getting closer and move on.
        self.declare_parameter('stuck_timeout', 20.0)
        self.declare_parameter('stuck_progress', 0.05)

        route_name = self.get_parameter('route').value
        if route_name not in ROUTES:
            raise SystemExit(f'unknown route {route_name}; '
                             f'have {sorted(ROUTES)}')
        self.waypoints = list(ROUTES[route_name])
        self.v_max = float(self.get_parameter('linear_speed').value)
        self.w_max = float(self.get_parameter('angular_speed').value)
        self.pos_tol = float(self.get_parameter('position_tolerance').value)
        self.head_tol = float(self.get_parameter('heading_tolerance').value)
        self.settle = float(self.get_parameter('settle_time').value)
        self.stuck_timeout = float(self.get_parameter('stuck_timeout').value)
        self.stuck_progress = float(self.get_parameter('stuck_progress').value)

        self.pose = None
        self.index = 0
        self.state = 'turn'
        self.state_since = None
        self.distance = 0.0
        self.last_xy = None
        self.finished = False
        self.best_distance = None
        self.best_at = None
        self.skipped = 0

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.on_odom, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'route {route_name}: {len(self.waypoints)} waypoints, '
            f'v={self.v_max} m/s w={self.w_max} rad/s')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))
        if self.last_xy is not None:
            self.distance += math.hypot(p.x - self.last_xy[0],
                                        p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def advance(self, why):
        self.index += 1
        self.state = 'turn'
        self.state_since = self.now()
        self.best_distance = None
        self.best_at = None
        self.publish(0.0, 0.0)
        if self.index < len(self.waypoints):
            self.get_logger().info(
                f'waypoint {self.index}/{len(self.waypoints)} ({why}) '
                f'-> {self.waypoints[self.index]} '
                f'({self.distance:.1f} m travelled)')

    def publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def tick(self):
        if self.pose is None or self.finished:
            return
        if self.index >= len(self.waypoints):
            self.publish(0.0, 0.0)
            if not self.finished:
                self.finished = True
                self.get_logger().info(
                    f'Route finished. travelled {self.distance:.1f} m, '
                    f'{self.skipped} waypoint(s) skipped')
            return

        x, y, th = self.pose
        tx, ty = self.waypoints[self.index]
        dx, dy = tx - x, ty - y
        distance = math.hypot(dx, dy)
        bearing = wrap(math.atan2(dy, dx) - th)

        if distance < self.pos_tol:
            self.advance('reached')
            return

        # Stuck watchdog: the target has to keep getting closer.
        now = self.now()
        if self.best_distance is None or distance < self.best_distance - self.stuck_progress:
            self.best_distance = distance
            self.best_at = now
        elif self.best_at is not None and now - self.best_at > self.stuck_timeout:
            self.get_logger().warn(
                f'waypoint {self.waypoints[self.index]} unreachable '
                f'({distance:.2f} m away, no progress for '
                f'{self.stuck_timeout:.0f} s) - skipping')
            self.skipped += 1
            self.advance('skipped')
            return

        if self.state == 'turn':
            if abs(bearing) > self.head_tol:
                # Ease off near the target heading so the rover does not
                # overshoot and oscillate.
                w = max(-self.w_max, min(self.w_max, 1.5 * bearing))
                if abs(w) < 0.08:
                    w = math.copysign(0.08, bearing)
                self.publish(0.0, w)
                self.state_since = None
                return
            # Heading reached: hold still briefly so the scan matcher gets a
            # stationary scan before translation starts.
            if self.state_since is None:
                self.state_since = self.now()
            self.publish(0.0, 0.0)
            if self.now() - self.state_since >= self.settle:
                self.state = 'drive'
            return

        # drive
        if abs(bearing) > 0.6:
            self.state = 'turn'
            self.state_since = None
            self.publish(0.0, 0.0)
            return
        v = min(self.v_max, max(0.08, 0.8 * distance))
        w = max(-self.w_max, min(self.w_max, 1.2 * bearing))
        self.publish(v, w)


def main():
    rclpy.init()
    node = ScriptedDrive()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(10):
            node.publish(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

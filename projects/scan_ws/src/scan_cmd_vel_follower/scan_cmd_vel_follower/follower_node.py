import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ScanCmdVelFollower(Node):
    def __init__(self) -> None:
        super().__init__("scan_cmd_vel_follower")

        self.path_topic = self.declare_parameter("path_topic", "/visual_local_trajectory").value
        self.odom_topic = self.declare_parameter("odom_topic", "/Odometry").value
        self.cmd_topic = self.declare_parameter("cmd_topic", "/cmd_vel").value
        self.planning_frame = self.declare_parameter("planning_frame", "camera_init").value

        self.lookahead_distance = float(self.declare_parameter("lookahead_distance", 0.6).value)
        self.goal_tolerance = float(self.declare_parameter("goal_tolerance", 0.25).value)
        self.max_linear_x = float(self.declare_parameter("max_linear_x", 0.08).value)
        self.max_angular_z = float(self.declare_parameter("max_angular_z", 0.18).value)
        self.k_linear = float(self.declare_parameter("k_linear", 0.35).value)
        self.k_angular = float(self.declare_parameter("k_angular", 0.9).value)
        self.slow_heading = float(self.declare_parameter("slow_heading", 0.7).value)
        self.stop_heading = float(self.declare_parameter("stop_heading", 1.2).value)
        self.path_timeout = float(self.declare_parameter("path_timeout", 1.0).value)
        self.odom_timeout = float(self.declare_parameter("odom_timeout", 0.5).value)
        self.publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)
        self.allow_reverse = bool(self.declare_parameter("allow_reverse", False).value)

        self.path_points: List[Tuple[float, float, float]] = []
        self.path_time: Optional[float] = None
        self.odom_pose: Optional[Tuple[float, float, float]] = None
        self.odom_time: Optional[float] = None
        self.last_status_time = 0.0

        data_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Path, self.path_topic, self.path_callback, data_qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, data_qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, data_qos)

        period = 1.0 / max(self.publish_rate, 1.0)
        self.create_timer(period, self.control_timer)

        self.get_logger().info(
            "SCAN cmd_vel follower ready: path=%s odom=%s cmd=%s frame=%s "
            "max_vx=%.3f max_wz=%.3f lateral=disabled"
            % (
                self.path_topic,
                self.odom_topic,
                self.cmd_topic,
                self.planning_frame,
                self.max_linear_x,
                self.max_angular_z,
            )
        )

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def path_callback(self, msg: Path) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.planning_frame:
            self.get_logger().warn(
                "Ignore path frame_id='%s', expected '%s'"
                % (msg.header.frame_id, self.planning_frame)
            )
            return

        points = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            )
            for pose in msg.poses
        ]
        self.path_points = points
        self.path_time = self.now_seconds()

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.planning_frame:
            self.get_logger().warn(
                "Ignore odom frame_id='%s', expected '%s'"
                % (msg.header.frame_id, self.planning_frame)
            )
            return

        q = msg.pose.pose.orientation
        self.odom_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )
        self.odom_time = self.now_seconds()

    def control_timer(self) -> None:
        now = self.now_seconds()
        twist = Twist()

        if not self.inputs_valid(now):
            self.cmd_pub.publish(twist)
            return

        assert self.odom_pose is not None
        x, y, yaw = self.odom_pose
        target = self.select_target(x, y)
        final = self.path_points[-1]

        if math.hypot(final[0] - x, final[1] - y) <= self.goal_tolerance:
            self.cmd_pub.publish(twist)
            self.status_throttle(now, "Goal reached, publish zero cmd.")
            return

        dx = target[0] - x
        dy = target[1] - y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x_body = cos_yaw * dx + sin_yaw * dy
        y_body = -sin_yaw * dx + cos_yaw * dy
        heading = normalize_angle(math.atan2(y_body, x_body))

        angular_z = clamp(self.k_angular * heading, -self.max_angular_z, self.max_angular_z)

        if abs(heading) > self.stop_heading:
            linear_x = 0.0
        else:
            forward_error = x_body if not self.allow_reverse else math.hypot(dx, dy)
            linear_x = self.k_linear * forward_error
            if abs(heading) > self.slow_heading:
                linear_x *= max(0.0, math.cos(heading))
            if not self.allow_reverse:
                linear_x = max(0.0, linear_x)
            linear_x = clamp(linear_x, -self.max_linear_x, self.max_linear_x)

        twist.linear.x = linear_x
        twist.linear.y = 0.0
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def inputs_valid(self, now: float) -> bool:
        if self.odom_pose is None or self.odom_time is None:
            self.status_throttle(now, "Waiting for odom.")
            return False
        if now - self.odom_time > self.odom_timeout:
            self.status_throttle(now, "Odom timeout, publish zero cmd.")
            return False
        if len(self.path_points) < 2 or self.path_time is None:
            self.status_throttle(now, "Waiting for local trajectory.")
            return False
        if now - self.path_time > self.path_timeout:
            self.status_throttle(now, "Path timeout, publish zero cmd.")
            return False
        return True

    def select_target(self, x: float, y: float) -> Tuple[float, float, float]:
        nearest_index = 0
        nearest_distance = float("inf")
        for i, point in enumerate(self.path_points):
            dist = math.hypot(point[0] - x, point[1] - y)
            if dist < nearest_distance:
                nearest_distance = dist
                nearest_index = i

        for point in self.path_points[nearest_index:]:
            if math.hypot(point[0] - x, point[1] - y) >= self.lookahead_distance:
                return point
        return self.path_points[-1]

    def status_throttle(self, now: float, message: str) -> None:
        if now - self.last_status_time > 2.0:
            self.get_logger().info(message)
            self.last_status_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanCmdVelFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

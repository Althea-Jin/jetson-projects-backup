import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class SafetyBridge(Node):
    def __init__(self):
        super().__init__('safety_bridge')

        # 先用比较保守的速度上限，真机时再根据机器人调整
        self.max_vx = 0.20       # m/s
        self.max_vy = 0.15       # m/s
        self.max_wz = 0.40       # rad/s

        # 超过 0.3 秒没有收到新速度，就自动停车
        self.timeout_sec = 0.30

        # 接收 MPC 的速度命令
        self.subscription = self.create_subscription(
            TwistStamped,
            '/mpc/cmd_vel',
            self.cmd_callback,
            10
        )

        # 现在故意不直接发布到机器人真正的 /cmd_vel
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel_safe',
            10
        )

        self.last_cmd_time = None
        self.safe_cmd = Twist()

        # 20 Hz 检查一次是否超时
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            'Safety bridge started: /mpc/cmd_vel -> /cmd_vel_safe'
        )

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def cmd_callback(self, msg):
        # 对速度进行安全限幅
              self.safe_cmd.linear.x = self.clamp(
            msg.twist.linear.x, -self.max_vx, self.max_vx
        )

              self.safe_cmd.linear.y = self.clamp(
            msg.twist.linear.y, -self.max_vy, self.max_vy
        )

              self.safe_cmd.angular.z = self.clamp(
            msg.twist.angular.z, -self.max_wz, self.max_wz
        )

        # 记录最近一次收到命令的时间
              self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        output = Twist()

        if self.last_cmd_time is not None:
            now = self.get_clock().now()
            elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

            if elapsed <= self.timeout_sec:
                output.linear.x = self.safe_cmd.linear.x
                output.linear.y = self.safe_cmd.linear.y
                output.angular.z = self.safe_cmd.angular.z

        # 如果超时，output 保持全 0，相当于自动停车
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)

    node = SafetyBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

if rclpy.ok():
    rclpy.shutdown()


if __name__ == '__main__':
    main()
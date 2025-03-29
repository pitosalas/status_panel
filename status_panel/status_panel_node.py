import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan
import time


class StatusPanelNode(Node):
    def __init__(self):
        super().__init__('odom_batt_scan_monitor')

        # Create subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(BatteryState, '/batt', self.batt_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # State variables
        self.odom_msg_count = 0
        self.scan_msg_count = 0
        self.current_voltage = None
        self.last_time = time.time()

        # Timer: fires at 1 Hz to calculate rates and call report_status
        self.create_timer(1.0, self.timer_callback)

    def odom_callback(self, msg):
        self.odom_msg_count += 1

    def batt_callback(self, msg):
        self.current_voltage = msg.voltage

    def scan_callback(self, msg):
        self.scan_msg_count += 1

    def timer_callback(self):
        now = time.time()
        elapsed = now - self.last_time
        odom_rate = self.odom_msg_count / elapsed if elapsed > 0 else 0.0
        scan_rate = self.scan_msg_count / elapsed if elapsed > 0 else 0.0

        # Reset counters and timer for the next interval
        self.last_time = now
        self.odom_msg_count = 0
        self.scan_msg_count = 0

        # Call your provided function with voltage, odom rate, and scan rate.
        self.report_status(self.current_voltage, odom_rate, scan_rate)

    def report_status(self, voltage, odom_rate, scan_rate):
        # Replace this placeholder with your actual function implementation.
        if voltage is None:
            self.get_logger().warn("Battery voltage not yet received.")
        else:
            self.get_logger().info(
                f"Voltage: {voltage:.2f} V, Odom Rate: {odom_rate:.2f} msgs/sec, Scan Rate: {scan_rate:.2f} msgs/sec"
            )

    def run(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = StatusPanelNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
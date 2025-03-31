import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan
import time

import sys
import time
import status_panel.qwiic_micro_oled_lib

class StatusPanelNode(Node):
    def __init__(self):
        super().__init__('status_panel_node')

        # Create subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(BatteryState, '/batt', self.batt_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # State variables
        self.odom_msg_count = 0
        self.scan_msg_count = 0
        self.current_voltage = "n/a"
        self.last_time = time.time()

        # Timer: fires at 1 Hz to calculate rates and call report_status
        self.create_timer(1.0, self.timer_callback)

        self.ole = self.prep_oled()


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

    def prep_oled(self):
        ole = status_panel.qwiic_micro_oled_lib.QwiicMicroOled(60)
        if not ole.connected:
            print("The Qwiic Micro OLED device isn't connected to the system. Please check your connection", \
                file=sys.stderr)
            return None
        ole.begin()
        # ole.clear(ole.ALL)
        return ole

    def report_status(self, volt, odom_rate, scan_rate):
        # self.ole.clear(self.ole.ALL)
        # self.ole.clear(self.ole.PAGE)  # Clear the display's buffer
        self.ole.set_cursor(0,0)
        self.ole.set_font_type(0)  # Set font type to 0 for the first line
        self.ole.print(f"VOLT: {volt}\n")
        self.ole.print(f"ODOM: {odom_rate}\n")
        self.ole.print(f"SCAN: {scan_rate}\n")

        self.ole.set_cursor(70, 10)
        self.ole.set_font_type(1)
        if volt == "n/a":
            self.ole.print("ERR")  # If voltage is not available, print "ERR")
        else:
            self.ole.print("OK")


        self.ole.display()  # Write the buffer to the display       

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
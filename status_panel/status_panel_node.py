import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time

import sys
import time
import status_panel.qwiic_micro_oled_lib
import math


ok_bitmap = [192, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 192, 255, 0, 224, 248, 252, 28, 14, 14, 14, 14, 14, 14, 28, 252, 248, 224, 0, 0, 254, 254, 254, 192, 224, 240, 248, 60, 30, 14, 6, 2, 0, 255, 255, 0, 7, 31, 63, 56, 120, 112, 112, 112, 112, 112, 56, 63, 31, 7, 0, 0, 127, 127, 127, 3, 1, 3, 15, 31, 62, 120, 112, 96, 0, 255, 7, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 7]

err_bitmap = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 1, 1, 249, 137, 137, 137, 137, 137, 1, 249, 9, 9, 9, 9, 241, 1, 1, 249, 9, 9, 9, 9, 241, 1, 1, 1, 255, 0, 0, 0, 0, 127, 64, 64, 95, 80, 80, 80, 80, 80, 64, 95, 65, 65, 65, 67, 76, 80, 64, 95, 65, 65, 65, 67, 76, 80, 64, 64, 127, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

class StatusPanelNode(Node):
    def __init__(self):
        super().__init__("status_panel_node")

        # Create subscriptions
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(BatteryState, "/battery", self.batt_callback, 10)

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(LaserScan, "/scan", self.scan_callback, qos)

        # State variables
        self.odom_msg_count = 0
        self.scan_msg_count = 0
        self.current_voltage = 0.0
        self.last_time = time.time()

        # Timer: fires at 1 Hz to calculate rates and call report_status
        self.create_timer(5.0, self.timer_callback)

        self.ole = self.prep_oled()

    def odom_callback(self, msg):
        self.odom_msg_count += 1

    def batt_callback(self, msg):
        self.current_voltage = float(msg.voltage)

    def scan_callback(self, msg):
        self.scan_msg_count += 1

    def timer_callback(self):
        now = time.time()
        elapsed = now - self.last_time
        self.odom_rate = self.odom_msg_count / elapsed if elapsed > 0 else 0.0
        self.scan_rate = self.scan_msg_count / elapsed if elapsed > 0 else 0.0

        # Reset counters and timer for the next interval
        self.last_time = now
        self.odom_msg_count = 0
        self.scan_msg_count = 0

        # Call your provided function with voltage, odom rate, and scan rate.
        self.report_status(self.current_voltage, self.odom_rate, self.scan_rate)
        self.show_status_icons()

    def show_status_icons(self):
        if not self.ole:
            return
        if (self.current_voltage < 10.0) or self.odom_rate < 10.0:
            # Display error icon
            self.display_error_icon()
        else:
            # Display OK icon
            self.display_ok_icon()

    def display_ok_icon(self):
        self.display_bitmap(
            bit_width=32,
            bit_height=32,
            start_x=96,
            start_y=0,
            bitmap=ok_bitmap
        )

    def display_error_icon(self):
        self.display_bitmap(
            bit_width=32,
            bit_height=32,
            start_x=96,
            start_y=0,
            bitmap=err_bitmap
        )


    def prep_oled(self):
        ole = status_panel.qwiic_micro_oled_lib.QwiicMicroOled(60)
        if not ole.connected:
            print(
                "The Qwiic Micro OLED device isn't connected to the system. Please check your connection",
                file=sys.stderr,
            )
            return None
        ole.begin()
        ole.clear(ole.ALL)
        ole.clear(ole.PAGE)  # Clear the display's buffer
        return ole

    def report_status(self, volt, odom_rate, scan_rate):
        self.ole.set_cursor(0, 0)
        self.ole.set_font_type(0)  # Set font type to 0 for the first line
        self.ole.print(f"VOLT: {volt:.2f}")
        self.ole.write("\n")  # New line
        self.ole.print(f"ODOM: {odom_rate:.2f}")
        self.ole.write("\n")  # New line
        self.ole.print(f"SCAN: {scan_rate:.2f}")
        self.ole.write("\n")  # New line
        self.ole.display()  # Write the buffer to the display


    def display_bitmap(self, bit_width, bit_height, start_x, start_y, bitmap):
        if self.ole:
            o = self.ole
            scrn_buffer = o._screenbuffer
            for page in range(math.ceil(bit_height / 8)):
                for col in range(bit_width):
                    screenpos = ((page + start_y) * 128) + col + start_x
                    bitmap_pos = page * bit_width + col
                    scrn_buffer[screenpos] = bitmap[bitmap_pos]
            o.display()

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


if __name__ == "__main__":
    main()

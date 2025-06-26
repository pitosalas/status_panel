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


ok_screen_buffer = [
    124,
    0,
    16,
    56,
    56,
    40,
    32,
    32,
    32,
    32,
    32,
    32,
    40,
    56,
    56,
    16,
    0,
    0,
    56,
    56,
    56,
    16,
    16,
    24,
    24,
    56,
    40,
    32,
    32,
    0,
    0,
    124,
    124,
    0,
    24,
    24,
    56,
    32,
    36,
    36,
    36,
    36,
    36,
    36,
    32,
    56,
    24,
    24,
    0,
    0,
    60,
    60,
    60,
    16,
    8,
    24,
    24,
    24,
    52,
    36,
    36,
    36,
    0,
    124,
    126,
    66,
    90,
    90,
    94,
    70,
    102,
    102,
    102,
    102,
    102,
    102,
    70,
    94,
    90,
    90,
    66,
    66,
    126,
    126,
    126,
    74,
    74,
    74,
    90,
    86,
    86,
    102,
    102,
    98,
    66,
    126,
    62,
    0,
    8,
    28,
    28,
    20,
    20,
    4,
    4,
    4,
    4,
    4,
    20,
    28,
    28,
    8,
    0,
    0,
    28,
    28,
    28,
    8,
    8,
    8,
    28,
    20,
    20,
    20,
    0,
    0,
    0,
    62,
]


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
        self.create_timer(1.0, self.timer_callback)

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
        odom_rate = self.odom_msg_count / elapsed if elapsed > 0 else 0.0
        scan_rate = self.scan_msg_count / elapsed if elapsed > 0 else 0.0

        # Reset counters and timer for the next interval
        self.last_time = now
        self.odom_msg_count = 0
        self.scan_msg_count = 0

        # Call your provided function with voltage, odom rate, and scan rate.
        # self.report_status(self.current_voltage, odom_rate, scan_rate)
        self.display_ok2()

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

    def display_ok2(self):

        if self.ole:
            o = self.ole
            scrn_buffer = o._screenbuffer
            scrn_width = 128
            scrn_height = 32
            start_x = 96
            start_y = 0
            for x in range(start_x, scrn_width):
                for y in range(math.ceil(start_y/8), math.ceil(scrn_height/8)):
                    if x < len(ok_screen_buffer) and y < len(ok_screen_buffer):
                        index = y * scrn_width + x
                        print(f"Setting pixel at ({x}, {y}) [{index}] to {ok_screen_buffer[x]}")
                        scrn_buffer[index] = ok_screen_buffer[y * math.ceil(scrn_width/8) + x]
            o.display()

    def dislay_ok(self):
        if self.ole:
            myOLED  = self.ole
            # myOLED.begin()

            #  clear(ALL) will clear out the OLED's graphic memory.
            # myOLED.clear(myOLED.ALL) #  Clear the display's memory (gets rid of artifacts)

            # #  Display buffer contents
            # myOLED.display()
            # time.sleep(3)

            #  clear(PAGE) will clear the Arduino's display buffer.
            # myOLED.clear(myOLED.PAGE)  #  Clear the display's buffer

            #  Display buffer contents
            # myOLED.display()
            # time.sleep(3)

            print(
                f"bitmap supplied length is {len(ok_screen_buffer)}... screen buffer of chip is: {len(myOLED._screenbuffer)}"
            )

            #  Draw Bitmap
            #  ---------------------------------------------------------------------------
            #  Add bitmap to buffer
            myOLED.draw_bitmap(ok_screen_buffer)

            #  To actually draw anything on the display, you must call the display() function.
            myOLED.display()

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

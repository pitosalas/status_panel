
import sys
import time
from status_panel.qwiic_micro_oled import qwiic_micro_oled

def prep_oled():
    myOLED = qwiic_micro_oled.QwiicMicroOled(60)
    if not myOLED.connected:
        print("The Qwiic Micro OLED device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return None
    myOLED.begin()
    myOLED.clear(myOLED.ALL)
    return myOLED


def display_variable(oled, title, value):
    oled.print(f"{title}: {value}\\n")

def display_screen(oled):
    oled.clear(oled.ALL)
    oled.clear(oled.PAGE)  # Clear the display's buffer
    display_variable(oled, "Time", time.strftime("%H:%M:%S"))
    display_variable(oled, "Date", time.strftime("%Y-%m-%d"))
    oled.display()  # Write the buffer to the display       

if __name__ == '__main__':
    try:
        ole = prep_oled()
        # display_screen(ole)
        ole.print("\n")
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding OLED Hello Example")
        sys.exit(0)

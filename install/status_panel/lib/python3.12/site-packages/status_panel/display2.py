
import sys
import time
import status_panel.qwiic_oled_base_py
import status_panel.qwiic_oled_base_py.qwiic_oled


#qwiic_micro_oled_lib.qwiic_micro_oled_base import QwiicOledBase



def prep_oled():
    print("******* Entering prep_oled ********")
    myOLED = status_panel.qwiic_oled_base_py.qwiic_oled.qwiic_micro_oled.QwiicMicroOled(address=0x3D)
    if not myOLED.connected:
        print("The Qwiic Micro OLED device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return None
    myOLED.set_font_type(2) 
    print("******* next: oled.begin() ********")
    myOLED.begin()
    print("******* next: oled.clear() ********")
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

def main():
    try:
        print("\nOLED Display - Hello World Example\n")
        ole = prep_oled()
        display_screen(ole)
        ole.print("\n")
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding OLED Hello Example")
        sys.exit(0)


if __name__ == '__main__':
    main()
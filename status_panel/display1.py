
import sys
import time
import status_panel.qwiic_oled_base_py
import status_panel.qwiic_oled_base_py.qwiic_oled


#qwiic_micro_oled_lib.qwiic_micro_oled_base import QwiicOledBase



def prep_oled():
    print("******* Entering prep_oled ********")
    myOLED = status_panel.qwiic_oled_base_py.qwiic_oled.QwiicOledDisplay(address=0x3C)
    if not myOLED.connected:
        print("The Qwiic Micro OLED device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return None
    myOLED.set_font_type(2 )
    print("******* next: oled.begin() ********")
    myOLED.begin()
    print("******* next: oled.clear() ********")
    myOLED.clear(myOLED.ALL)
    return myOLED

def display_variable(oled, title, value):
    oled.print(f"{title}: {value}\\n")

def display_screen(myOLED):
# Demonstrate font 1. 8x16. Let's use the print function
    # to display every character defined in this font.
    myOLED.set_font_type(2)  # Set font to type 1
    myOLED.clear(myOLED.PAGE)     # Clear the page
    myOLED.set_cursor(0, 0) # Set cursor to top-left
    # Print can be used to print a string to the screen:
    myOLED.print(" !\"#$%&'()*+,-./01234")
    myOLED.display()       # Refresh the display
    time.sleep(1)

    myOLED.clear(myOLED.PAGE)
    myOLED.set_cursor(0, 0)
    myOLED.print("OK")
    myOLED.display()
    time.sleep(1)

def show_fonts(myOLED):
    for font in range(6):
        myOLED.set_font_type(font)
        print(f"Font {font}")
        myOLED.clear(myOLED.PAGE)
        myOLED.set_cursor(0, 0)
        myOLED.print(f"OK{font}")
        myOLED.display()
        time.sleep(2)

def main(): 
    try:
        print("\nOLED Display - Hello World Example\n")
        ole = prep_oled()
        # display_screen(ole)
        show_fonts(ole)
        ole.print("\n")
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding OLED Hello Example")
        sys.exit(0)


if __name__ == '__main__':
    main()
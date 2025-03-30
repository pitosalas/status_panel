import sys
import status_panel.test_lib.test_funcs


def main():
    print("Main called")
    status_panel.test_lib.test_funcs.test_func()  # Call the function from the test_lib module
    sys.exit(0)


if __name__ == '__main__':
    main()
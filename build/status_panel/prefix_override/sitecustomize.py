import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pitosalas/linorobot2_ws/src/status_panel/install/status_panel'

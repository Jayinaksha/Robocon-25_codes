import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/noblehalogen/joy_serial_ws/install/joy_serial_bridge'

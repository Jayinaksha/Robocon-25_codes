import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/noblehalogen/Robocon25_codes/install/joy_serial_bridge'

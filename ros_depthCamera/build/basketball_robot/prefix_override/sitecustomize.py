import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspace/Robocon25_codes/ros_depthCamera/install/basketball_robot'

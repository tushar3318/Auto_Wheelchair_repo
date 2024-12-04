import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vikas_maurya/ros2_ws/src/Auto_Wheelchair_repo/install/wc_robot'

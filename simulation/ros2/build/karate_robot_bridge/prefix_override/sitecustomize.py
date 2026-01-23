import sys
if sys.prefix == '/Users/jeffboggs/miniforge3/envs/ros2':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/jeffboggs/karate_robot/simulation/ros2/install/karate_robot_bridge'

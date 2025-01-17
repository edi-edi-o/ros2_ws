# ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash

ros2 launch imu_phone_data imu_launch.py

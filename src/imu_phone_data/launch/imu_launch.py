from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    # Path to the URDF file
    urdf_path = PathJoinSubstitution([
        '/home/edi/ros2_ws/src/imu_phone_data/urdf',
        'imu_model.urdf'
    ])

    # Read the URDF file content
    with open(os.path.join('/home/edi/ros2_ws/src/imu_phone_data/urdf', 'imu_model.urdf'), 'r') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        Node(
            package='imu_phone_data',
            executable='imu_node',
            name='imu_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'debug']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/edi/ros2_ws/src/imu_phone_data/rviz/imu_display.rviz']
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        )
    ])
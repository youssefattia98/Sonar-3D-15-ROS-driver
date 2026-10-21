import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix

def generate_launch_description():
    python_site_packages = PathJoinSubstitution([
        FindPackagePrefix('sonar3d'),
        'lib',
        f'python{sys.version_info.major}.{sys.version_info.minor}',
        'site-packages',
    ])

    return LaunchDescription([
        # Make the Python node independent of the terminal's cached environment.
        # This is needed when sonar3d was built after the workspace was sourced.
        SetEnvironmentVariable(
            'PYTHONPATH',
            [
                python_site_packages,
                os.pathsep,
                EnvironmentVariable('PYTHONPATH', default_value=''),
            ],
        ),
        DeclareLaunchArgument('sonar_ip', default_value='192.168.2.197'),
        DeclareLaunchArgument('local_interface_ip', default_value='192.168.2.1'),
        DeclareLaunchArgument('speed_of_sound', default_value='1491'),
        DeclareLaunchArgument('sync_time', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('correct_sonar_clock_offset', default_value='true'),
        DeclareLaunchArgument('sonar_frame', default_value='sonar_frame'),
        DeclareLaunchArgument('imu_frame', default_value='sonar_imu_frame'),
        Node(
            package='sonar3d',
            executable='sonar_publisher',
            name='sonar_node',
            output='screen',
            parameters=[
                        {'IP': LaunchConfiguration('sonar_ip')},
                        {'local_interface_ip': LaunchConfiguration('local_interface_ip')},
                        {'speed_of_sound': LaunchConfiguration('speed_of_sound')},
                        {'sync_time_on_startup': LaunchConfiguration('sync_time')},
                        {'enable_imu': LaunchConfiguration('enable_imu')},
                        {'correct_sonar_clock_offset': LaunchConfiguration('correct_sonar_clock_offset')},
                        {'sonar_frame': LaunchConfiguration('sonar_frame')},
                        {'imu_frame': LaunchConfiguration('imu_frame')},
                    ]
        )
    ])

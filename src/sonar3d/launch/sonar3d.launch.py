import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def sonar_pythonpath():
    """Return a PYTHONPATH that also works with a stale sourced workspace."""
    site_packages = (
        Path(get_package_prefix('sonar3d'))
        / 'lib'
        / f'python{sys.version_info.major}.{sys.version_info.minor}'
        / 'site-packages'
    )

    paths = []
    # colcon --symlink-install represents this package with an egg-link. Python's
    # import machinery only follows it when the workspace setup script has added
    # the linked build directory, so add that directory explicitly as well.
    egg_link = site_packages / 'sonar3d.egg-link'
    if egg_link.is_file():
        for line in egg_link.read_text(encoding='utf-8').splitlines():
            candidate = line.strip()
            if candidate and candidate != '.':
                paths.append(candidate)
                break

    paths.append(str(site_packages))
    existing_pythonpath = os.environ.get('PYTHONPATH')
    if existing_pythonpath:
        paths.append(existing_pythonpath)
    return os.pathsep.join(paths)


def generate_launch_description():
    return LaunchDescription([
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
            additional_env={'PYTHONPATH': sonar_pythonpath()},
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

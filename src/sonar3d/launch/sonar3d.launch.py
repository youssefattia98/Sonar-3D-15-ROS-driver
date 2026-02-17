from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sonar3d',
            executable='sonar_publisher',
            name='sonar_node',
            output='screen',
            parameters=[
                        {'IP': '192.168.2.50'},                 # sonar IP (filter / API)
                        {'local_interface_ip': '192.168.2.1'},  # host Ethernet IP on enp0s31f6
                        {'speed_of_sound': 1445},
                    ]
        )
    ])

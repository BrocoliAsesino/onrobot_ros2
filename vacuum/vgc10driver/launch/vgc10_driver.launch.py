from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vgc10driver',
            executable='vgc10_ros2_driver',
            name='vgc10_driver',
            parameters=[{
                'ip_address': '192.168.1.1',
                'port': 502,
                'slave_id': 65,
                'use_dummy': True
            }],
            output='screen'
        )
    ])
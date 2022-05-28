from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='carl-ros2foxy-baseline',
            namespace='carl1'
            executable='arduino_comm',
            name='my_carl'
        )
    ])
        

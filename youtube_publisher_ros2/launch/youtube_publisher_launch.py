import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='youtube_publisher_ros2',
            executable='youtube_publisher_node',  # matches the entry point in setup.py
            name='youtube_publisher_node',
            output='screen',
            parameters=[{'youtube_link': 'https://www.youtube.com/watch?v=SAanKlcupmI'}]
        )
    ])

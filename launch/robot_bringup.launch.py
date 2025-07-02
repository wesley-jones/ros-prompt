from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_prompt',
            executable='capability_scanner_node',
            name='capability_scanner',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('ros_prompt'), 'config', 'robot_caps.yaml')]
        ),
        Node(
            package='ros_prompt',
            executable='llm_interface_node',
            name='llm_interface',
            output='screen',
            # parameters, remappings, etc.
        ),
        # Add more nodes as needed
    ])

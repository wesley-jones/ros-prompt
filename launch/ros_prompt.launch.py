from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([        
        Node(
            package='ros_prompt',
            executable='intent_node',
            name='intent_node',
            output='screen',
            # parameters, remappings, etc.
        ),
        Node(
            package='ros_prompt',
            executable='planner_node',
            name='planner_node',
            output='screen',
            # parameters, remappings, etc.
        ),
        Node(
            package='ros_prompt',
            executable='capability_scanner_node',
            name='capability_scanner',
            output='screen',
            #parameters=[os.path.join(get_package_share_directory('ros_prompt'), 'config', 'robot_caps.yaml')]
        ),
    ])

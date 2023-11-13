import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    
    return LaunchDescription([
        # Node(
        #     package='nav_pkg',
        #     executable='initial_position',
        #     output='screen',
        #     name="initial_position",
        # ),

        Node(
            package='nav_pkg',
            executable='input',
            output='screen',
            prefix = 'gnome-terminal --',
            name="input",
        ),
    
        Node(
            package='nav_pkg',
            executable='nav_node',
            output='screen',
            name="nav_node",
        ),
    ])

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    
    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    gazebo_dir + '/launch/turtlebot3_world.launch.py'))
    
    rviz_dir = get_package_share_directory('turtlebot3_cartographer')
    rviz = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(rviz_dir + '/launch/cartographer.launch.py'),
                launch_arguments={
                   "use_sim_time": 'true',
                }.items()
            )

    
    return LaunchDescription([
        gazebo,
        rviz,
        Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            output='screen',
            emulate_tty=True,
            prefix = 'gnome-terminal --',
            name="teleop"
        ),
    ])

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_gazebo'),
                    'launch',
                    'turtlebot3_world.launch.py'
                ])
            ]),
        ),
        
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_cartographer'),
                    'launch',
                    'cartographer.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'True'
            }.items()
        ),
        
        Node(
            package='my_robot_controller',
            namespace='mapping',
            executable='mapping',
            name='sim_mapping'
        ),
        
        
    ])
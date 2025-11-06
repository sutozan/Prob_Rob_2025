import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('color', default_value='cyan',
                              description='set color for topic name'),
        DeclareLaunchArgument('height', default_value='0.5',
                              description='height of landmark'),
        Node(
            package='prob_rob_labs',
            executable='measurment_calculations',
            name='measurment_calculations',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'color': LaunchConfiguration('color')},
                        {'height': LaunchConfiguration('height')}]
        )
    ])

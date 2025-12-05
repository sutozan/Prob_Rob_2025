import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('map_path', default_value='~/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/config/landmarks.json',
                              description='set path to map file'),
        DeclareLaunchArgument('height', default_value='0.5',
                              description='height of landmark'),
        DeclareLaunchArgument('dx_camera', default_value='0.076',
                              description='set camera offset in x-axis '),
        DeclareLaunchArgument('dy_camera', default_value='0.0',
                              description='set camera offset in y-axis'),
        Node(
            package='prob_rob_labs',
            executable='ekf_localization',
            name='ekf_localization',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'map_path': LaunchConfiguration('map_path')},
                        {'height': LaunchConfiguration('height')},
                        {'dx_camera': LaunchConfiguration('dx_camera')},
                        {'dy_camera': LaunchConfiguration('dy_camera')},]
        )
    ])

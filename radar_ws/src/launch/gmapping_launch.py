#!/usr/bin/env python3

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('rplidar_ros'), 'rviz', 'sllidar_gmapping_ros2.rviz')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='False')
    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            parameters=None,
            remappings=None,
            arguments=['-d', rviz_config_dir],
            output='screen',
        ),
        Node(
            package='slam_gmapping',
            namespace='transform',
            executable='transform',
            output='screen',
            parameters=[{'parents_frame': "odom",
                         'child_frame': "laser",
                         'x': 0.1,
                         'y': 0.2,
                         'z': 0.5,
                         'roll': 0.0,
                         'pitch': 0.0,
                         'yaw': 0.0}],
        ),
         Node(
            package='tf2_ros',
            namespace='laser_to_base_link',
            executable='static_transform_publisher',
            output='screen',
            arguments = [ '0', 
            			  '0', 
            			 '1', 
            			 '0', 
            			  '0', 
            			  '0', 
            			  'laser', 
            			 'base_link']

        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node 
import os
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource,AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
	LIDAR_TYPE = os.getenv('LIDAR_TYPE')
	print("my_lidar:",LIDAR_TYPE)
	lidar_type_arg = DeclareLaunchArgument(name='lidar_type', default_value=LIDAR_TYPE, 
                                              description='The type of lidar')
	a1_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
        '/rplidar_a1_launch.py']),
         condition=LaunchConfigurationEquals('lidar_type', 'a1')
    )
	s2_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
        '/rplidar_s2_launch.py']),
         condition=LaunchConfigurationEquals('lidar_type', 's2')
    )
	c1_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
        '/rplidar_c1_launch.py']),
         condition=LaunchConfigurationEquals('lidar_type', 'c1')
    )
	a3_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
        '/rplidar_a3_launch.py']),
         condition=LaunchConfigurationEquals('lidar_type', 'a3')
    )
	a2_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
        '/rplidar_a2m12_launch.py']),
         condition=LaunchConfigurationEquals('lidar_type', 'a2')
    )
	return LaunchDescription([
        lidar_type_arg,
        a1_launch,
        s2_launch,
        c1_launch,
        a3_launch,
        a2_launch
    ])

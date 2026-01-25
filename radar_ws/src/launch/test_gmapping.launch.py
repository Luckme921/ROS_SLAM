from launch import LaunchDescription
from launch_ros.actions import Node 
import os
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals,LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource,AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
	LIDAR_TYPE = os.getenv('LIDAR_TYPE')
	lidar_type_arg = DeclareLaunchArgument(name='lidar_type', default_value=LIDAR_TYPE, description='The type of lidar')
	
	lidar_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),'/lidar.launch.py']))
	
	s2_gmapping_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_gmapping'), 'launch'),'/s2_gmapping_launch.py']),condition=LaunchConfigurationEquals('lidar_type', 's2'))
	
	gmapping_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_gmapping'), 'launch'),'/gmapping_launch.py']),condition=LaunchConfigurationNotEquals('lidar_type', 's2'))
	
	s2_slam_gmapping_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_gmapping'), 'launch'),'/s2_slam_gmapping.launch.py']),condition=LaunchConfigurationEquals('lidar_type', 's2'))
	
	slam_gmapping_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_gmapping'), 'launch'),'/slam_gmapping.launch.py']),condition=LaunchConfigurationNotEquals('lidar_type', 's2'))
	
	
	return LaunchDescription([
        lidar_type_arg,
        lidar_launch,
        s2_gmapping_launch,
        s2_slam_gmapping_launch,
        gmapping_launch,
        slam_gmapping_launch
    ])
                               

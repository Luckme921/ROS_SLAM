import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # ====================== 路径配置 ======================
    # 1. 被启动的launch（mc_full.launch）相关路径
    mc_chassis_dir = get_package_share_directory('mc_chassis')
    mc_full_launch_path = os.path.join(mc_chassis_dir, 'launch', 'mc_full.launch.py') 
    
    # 2. nav2相关路径
    slam_nav2_dir = get_package_share_directory('slam_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 3. Launch参数配置
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(slam_nav2_dir, 'maps', 'room1.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(slam_nav2_dir, 'config', 'nav2_params.yaml'))

    # ====================== 核心逻辑 ======================
    # 1. 最先启动：被依赖的mc_full.launch
    mc_full_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mc_full_launch_path),
        # 如果需要给mc_full.launch传参，可以在这里添加launch_arguments
        # launch_arguments={'serial_port': '/dev/ttyUSB0'}.items(),
    )

    # 2. 延时3秒后执行的动作（nav2 + rviz2）
    delayed_actions = launch.actions.TimerAction(
        period=3.0,  # 延时3秒
        actions=[
            # nav2 bringup启动
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
                launch_arguments={
                    'map': map_yaml_path,
                    'params_file': nav2_param_path
                }.items(),
            ),
            # rviz2节点
            launch_ros.actions.Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_dir],
                output='screen'
            )
        ]
    )

    # ====================== 组合并返回 ======================
    return launch.LaunchDescription([
        # 声明参数
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),
        
        # 启动顺序：先mc_full.launch，再延时执行后续动作
        mc_full_launch,
        delayed_actions
    ])
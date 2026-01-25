import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.parameter_descriptions

def generate_launch_description():
    # ====================== 1. 机器人模型配置（保留原有路径） ======================
    # 获取urdf_show包的共享目录
    urdf_show_path = get_package_share_directory('urdf_show')
    # 机器人URDF文件路径（原有路径）
    default_robot_model_path = os.path.join(urdf_show_path, 'urdf', 'urdf_mac.urdf')
    # 机器人默认RViz配置路径（你原有路径，无需修改）
    default_rviz_config_path = os.path.join(urdf_show_path, 'config_rviz', 'display_robot_model.rviz')
    
    # 声明机器人参数
    declare_robot_model_arg = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_robot_model_path),
        description='机器人URDF文件绝对路径'
    )
    # 生成机器人描述参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )

    # ====================== 2. 雷达配置 ======================
    # 雷达核心参数
    declare_channel_type_arg = launch.actions.DeclareLaunchArgument(
        'channel_type', default_value='serial', description='雷达通信类型'
    )
    declare_serial_port_arg = launch.actions.DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_4d851953e10a6c41a0e589766753218c-if00-port0',
        description='雷达串口路径（保留你原有路径）'
    )
    declare_serial_baudrate_arg = launch.actions.DeclareLaunchArgument(
        'serial_baudrate', default_value='256000', description='雷达波特率（保留你原有值）'
    )
    # frame_id设为URDF中的lidar_Link
    declare_frame_id_arg = launch.actions.DeclareLaunchArgument(
        'frame_id', default_value='lidar_Link', description='雷达坐标系（匹配URDF的lidar_Link）'
    )
    declare_inverted_arg = launch.actions.DeclareLaunchArgument(
        'inverted', default_value='false', description='是否反转扫描数据'
    )
    declare_angle_compensate_arg = launch.actions.DeclareLaunchArgument(
        'angle_compensate', default_value='true', description='是否启用角度补偿'
    )
    declare_scan_mode_arg = launch.actions.DeclareLaunchArgument(
        'scan_mode', default_value='Sensitivity', description='雷达扫描模式'
    )

    # ====================== 3. 节点定义（只启动一个RViz，用原有配置） ======================
    # 机器人状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
    # 雷达驱动节点
    rplidar_node = launch_ros.actions.Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[
            {'channel_type': launch.substitutions.LaunchConfiguration('channel_type')},
            {'serial_port': launch.substitutions.LaunchConfiguration('serial_port')},
            {'serial_baudrate': launch.substitutions.LaunchConfiguration('serial_baudrate')},
            {'frame_id': launch.substitutions.LaunchConfiguration('frame_id')},  # lidar_Link
            {'inverted': launch.substitutions.LaunchConfiguration('inverted')},
            {'angle_compensate': launch.substitutions.LaunchConfiguration('angle_compensate')},
            {'scan_mode': launch.substitutions.LaunchConfiguration('scan_mode')}
        ],
        output='screen'
    )
    # RViz节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config_path], 
        output='screen'
    )

    # ====================== 4. 组合所有组件 ======================
    return launch.LaunchDescription([
        # 机器人参数
        declare_robot_model_arg,
        # 雷达参数
        declare_channel_type_arg,
        declare_serial_port_arg,
        declare_serial_baudrate_arg,
        declare_frame_id_arg,
        declare_inverted_arg,
        declare_angle_compensate_arg,
        declare_scan_mode_arg,
        # 启动节点
        joint_state_publisher_node,
        robot_state_publisher_node,
        rplidar_node,
        rviz_node
    ])
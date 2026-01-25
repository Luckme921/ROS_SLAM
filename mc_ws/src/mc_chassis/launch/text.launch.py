import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ====================== 路径配置 ======================
    urdf_show_pkg_dir = get_package_share_directory('urdf_show')
    default_robot_urdf_path = os.path.join(urdf_show_pkg_dir, 'urdf', 'urdf_mac.urdf')
    default_rviz_config_path = os.path.join(urdf_show_pkg_dir, 'config_rviz', 'display_robot_model.rviz')

    complementary_filter_launch_path = '/home/pi/imu_tools_catkin_ws/src/imu_tools/imu_complementary_filter/launch/complementary_filter.launch.py'
    
    # ====================== 参数声明 ======================
    declare_robot_model_arg = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=default_robot_urdf_path,
        description='机器人URDF文件绝对路径'
    )
    declare_channel_type_arg = launch.actions.DeclareLaunchArgument('channel_type', default_value='serial')
    declare_serial_port_arg = launch.actions.DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_4d851953e10a6c41a0e589766753218c-if00-port0'
    )
    declare_serial_baudrate_arg = launch.actions.DeclareLaunchArgument('serial_baudrate', default_value='256000')
    declare_frame_id_arg = launch.actions.DeclareLaunchArgument('frame_id', default_value='lidar_Link')
    declare_inverted_arg = launch.actions.DeclareLaunchArgument('inverted', default_value='false')
    declare_angle_compensate_arg = launch.actions.DeclareLaunchArgument('angle_compensate', default_value='true')
    declare_scan_mode_arg = launch.actions.DeclareLaunchArgument('scan_mode', default_value='Sensitivity')

    # 机器人描述参数
    robot_description = launch_ros.descriptions.ParameterValue(
        launch.substitutions.Command(['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )

    # ====================== 节点配置 ======================
    # 1. 机器人状态发布节点（核心TF节点）
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'publish_frequency': 50.0}],
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN']
    )

    # 2. 雷达节点
    rplidar_node = launch_ros.actions.Node(
        package='rplidar_ros',
        executable='rplidar_node',
        parameters=[
            {'channel_type': launch.substitutions.LaunchConfiguration('channel_type')},
            {'serial_port': launch.substitutions.LaunchConfiguration('serial_port')},
            {'serial_baudrate': launch.substitutions.LaunchConfiguration('serial_baudrate')},
            {'frame_id': launch.substitutions.LaunchConfiguration('frame_id')},
            {'inverted': launch.substitutions.LaunchConfiguration('inverted')},
            {'angle_compensate': launch.substitutions.LaunchConfiguration('angle_compensate')},
            {'scan_mode': launch.substitutions.LaunchConfiguration('scan_mode')}
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    # 3. 小车控制节点
    serial_comm_node = launch_ros.actions.Node(
        package='mc_chassis',
        executable='serial_comm_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    odom_node = launch_ros.actions.Node(
        package='mc_chassis',
        executable='odometry_publisher',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

  
    chassis_controller_node = launch_ros.actions.Node(
        package='mc_chassis',
        executable='chassis_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'] 
    )

    # 4. 轮子关节状态发布节点
    wheel_joint_states_node = launch_ros.actions.Node(
        package='mc_chassis',
        executable='wheel_joint_states_publisher',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    # 5. RViz节点
    rviz_cmd = [
        'bash', '-c', 
        f'sleep 5 && /opt/ros/humble/lib/rviz2/rviz2 -d {default_rviz_config_path} --ros-args --log-level WARN'
    ]
    rviz_node = launch.actions.ExecuteProcess(
        cmd=rviz_cmd,
        output='screen',
        name='rviz2'
    )
    complementary_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(complementary_filter_launch_path),
        # 可选：如果需要给IMU滤波传参数，取消下面注释并修改
        # launch_arguments={
        #     'imu_topic': '/imu/data', 
        #     'rate': '100.0'            # 滤波频率
        # }.items()
    )


    # ====================== 组合节点 ======================
    return launch.LaunchDescription([
        declare_robot_model_arg,
        declare_channel_type_arg,
        declare_serial_port_arg,
        declare_serial_baudrate_arg,
        declare_frame_id_arg,
        declare_inverted_arg,
        declare_angle_compensate_arg,
        declare_scan_mode_arg,

        # 启动顺序
        robot_state_publisher,
        rplidar_node,
        complementary_filter_launch,
        serial_comm_node,
        odom_node,
        chassis_controller_node,
        wheel_joint_states_node,
        rviz_node
    ])
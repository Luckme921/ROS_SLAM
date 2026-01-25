import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.parameter_descriptions


def generate_launch_description():
    # 获取默认路径
    urdf_tutorial_path = get_package_share_directory('urdf_show')
    # default_model_path = urdf_tutorial_path + '/urdf/first_robot.urdf' 
    default_model_path=os.path.join(urdf_tutorial_path,'urdf','urdf_mac.urdf')
    default_rviz_config_path = urdf_tutorial_path + '/config_rviz/display_robot_model.rviz'
    # 为 Launch 声明参数 urdf 路径
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),#model为launch的参数名，默认值为default_model_path即urdf_mac.urdf的路径
        description='URDF 的绝对路径')
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['cat ', launch.substitutions.LaunchConfiguration('model')]),#z注意xacro、cat后要有空格
        value_type=str)
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    # RViz 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]#也可以不传路径，但是需要手动配置rivz的配置
    )
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
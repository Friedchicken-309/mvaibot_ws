import launch
import launch_ros
from  ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #获取urdf默认路径
    urdf_tutorial_path = get_package_share_directory('mvaibot_description')
    default_model_path = urdf_tutorial_path + '/urdf/mvaibot.urdf'

    #获取RViz默认配置路径
    rviz_default_path = urdf_tutorial_path + '/config/rviz/display_model.rviz'

    #为launch声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_model_path),
        description='urdf的绝对路径')
    
    #获取文件内容并生成参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
    #状态发布节点
    robot_status_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    #关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
    #RViz显示节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_default_path],
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        joint_state_publisher_node,
        robot_status_publisher_node,
        rviz_node
    ])
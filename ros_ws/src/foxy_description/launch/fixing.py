import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
    pkg_share = FindPackageShare(package='foxy_description').find('foxy_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'omo.urdf'
    urdf = os.path.join(
        get_package_share_directory('foxy_description'),
        'urdf',
        urdf_file_name)
    gui = LaunchConfiguration('gui')

    rviz_config_dir = os.path.join(get_package_share_directory('foxy_description'),
                                   'rviz')        
    default_gazebo_config =os.path.join(pkg_share, 'world/world12.sdf')

    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    model = LaunchConfiguration('model')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')
 
    declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=urdf, 
    description='Absolute path to robot urdf file')

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    name='gui',
    default_value='True',
    description='Flag to enable joint_state_publisher_gui')


    start_joint_state_publisher_cmd = Node(
    # condition=UnlessCondition(gui),
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    remappings=[("/robot_description", "/unicycle_bot_robot_description")]
    )
    spawn_entity =Node(
  package='gazebo_ros',
  executable='spawn_entity.py',
  arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
  output='screen'
  )
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
)


    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        start_joint_state_publisher_cmd,
        declare_use_robot_state_pub_cmd,
        declare_model_path_cmd,
        robot_localization_node,
        spawn_entity,
        # declare_use_joint_state_publisher_cmd,

        Node(
            condition=IfCondition(use_robot_state_pub),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time, 
            'robot_description': Command(['xacro ', model])}],
            remappings=remappings,
            arguments=[urdf]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map'],
    ),
    ])
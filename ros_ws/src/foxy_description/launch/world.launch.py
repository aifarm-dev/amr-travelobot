import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = FindPackageShare(package='foxy_description').find('foxy_description')

    # world_file_name = 'world12.sdf'
    world = os.path.join(get_package_share_directory('foxy_description'),
                         'world', 'world12.sdf')
    launch_file_dir = os.path.join(get_package_share_directory('foxy_description'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
)
#     spawn_entity =Node(
#   package='gazebo_ros',
#   executable='spawn_entity.py',
#   arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
#   output='screen'
#   )
    return LaunchDescription([
#         Node(
#        package='robot_localization',
#        executable='ekf_node',
#        name='ekf_filter_node',
#        output='screen',
#        parameters=[os.path.join('foxy_description', 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
# ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([launch_file_dir, '/omo_r1mini_launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time}.items(),
        # ),
        # spawn_entity,
        #  robot_localization_node,

    ])
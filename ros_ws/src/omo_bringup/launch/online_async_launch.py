import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("omo_bringup"),
                                   'param', 'omo_r1mini_lidar.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
# import os
# from launch import LaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.actions import Node

# def generate_launch_description():

#     cartographer_config_dir = os.path.join(get_package_share_directory('ros2_mapping'), 'config')
#     configuration_basename = 'cartographer.lua'

#     return LaunchDescription([
        
#         Node(
#             package='cartographer_ros', 
#             executable='cartographer_node', 
#             name='cartographer_node',
#             output='screen',
#             parameters=[{'use_sim_time': True}],
#             arguments=['-configuration_directory', cartographer_config_dir,
#                        '-configuration_basename', configuration_basename]),

#         Node(
#             package='cartographer_ros',
#             executable='occupancy_grid_node',
#             output='screen',
#             name='occupancy_grid_node',
#             parameters=[{'use_sim_time': True}],
#             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
#         ),
#     ]) 
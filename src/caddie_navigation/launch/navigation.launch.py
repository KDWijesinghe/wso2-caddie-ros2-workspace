import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('caddie_navigation')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    default_slam_params = os.path.join(nav_pkg, 'config', 'slam_toolbox.yaml')
    default_nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(nav_pkg, 'rviz', 'caddie_nav.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_rviz = LaunchConfiguration('use_rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_slam),
    )

    slam_lifecycle = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': ['slam_toolbox'],
                    'bond_timeout': 0.0,
                }],
            )
        ],
        condition=IfCondition(use_slam),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
        }.items(),
        condition=IfCondition(use_nav2),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulated time'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically configure and activate lifecycle nodes'),
        DeclareLaunchArgument(
            'use_slam', default_value='true',
            description='Start SLAM Toolbox mapping'),
        DeclareLaunchArgument(
            'use_nav2', default_value='true',
            description='Start Nav2 navigation servers'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with caddie navigation layout'),
        DeclareLaunchArgument(
            'slam_params_file', default_value=default_slam_params,
            description='SLAM Toolbox parameter file'),
        DeclareLaunchArgument(
            'nav2_params_file', default_value=default_nav2_params,
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='RViz configuration file'),
        slam_node,
        slam_lifecycle,
        nav2,
        rviz,
    ])

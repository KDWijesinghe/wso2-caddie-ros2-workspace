import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory('caddie_description')
    gazebo_pkg = get_package_share_directory('caddie_gazebo')
    go2_description_pkg = get_package_share_directory('go2_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    default_model = os.path.join(
        description_pkg, 'urdf', 'go2_caddie_official.urdf.xacro')
    default_world = os.path.join(
        gazebo_pkg, 'worlds', 'wso2_caddie_course.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_leg_animation = LaunchConfiguration('use_leg_animation')
    leg_joint_type = PythonExpression([
        "'revolute' if '", use_leg_animation,
        "'.lower() == 'true' else 'fixed'",
    ])

    robot_description = ParameterValue(
        Command([
            'xacro ', model,
            ' use_gazebo:=true',
            ' leg_joint_type:=', leg_joint_type,
            ' use_leg_control:=', use_leg_animation,
        ]),
        value_type=str,
    )

    gz_args = [world, ' -r']
    go2_resource_path = os.path.dirname(go2_description_pkg)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(gui),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [world, ' -r -s'],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    hidden_wheel_joint_state_publisher = Node(
        package='caddie_control',
        executable='hidden_wheel_joint_state_publisher',
        name='hidden_wheel_joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_joint_state_broadcaster',
        output='screen',
        condition=IfCondition(use_leg_animation),
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
    )

    go2_leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_go2_leg_controller',
        output='screen',
        condition=IfCondition(use_leg_animation),
        arguments=[
            'go2_leg_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
    )

    go2_gait_animator = Node(
        package='caddie_control',
        executable='go2_gait_animator',
        name='go2_gait_animator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_leg_animation),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'go2_caddie',
            '-allow_renaming', 'true',
            '-x', '-15.0',
            '-y', '-7.0',
            '-z', '0.54',
            '-Y', '0.20',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/scan', '/gz/scan'),
            ('/imu', '/gz/imu'),
            ('/camera/image', '/gz/camera/image'),
            ('/camera/depth_image', '/gz/camera/depth_image'),
            ('/camera/camera_info', '/gz/camera/camera_info'),
        ],
    )

    sensor_frame_normalizer = Node(
        package='caddie_perception',
        executable='sensor_frame_normalizer',
        name='sensor_frame_normalizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulated time'),
        DeclareLaunchArgument(
            'model', default_value=default_model,
            description='Path to the Go2 caddie xacro file'),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Path to the Gazebo world SDF'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start Gazebo with GUI'),
        DeclareLaunchArgument(
            'use_leg_animation', default_value='false',
            description='Enable experimental visible Go2 leg gait animation'),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=go2_resource_path,
            prepend=True,
            separator=':'),
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=go2_resource_path,
            prepend=True,
            separator=':'),
        gz_sim,
        gz_sim_headless,
        robot_state_publisher,
        hidden_wheel_joint_state_publisher,
        spawn_robot,
        joint_state_broadcaster_spawner,
        go2_leg_controller_spawner,
        go2_gait_animator,
        bridge,
        sensor_frame_normalizer,
    ])

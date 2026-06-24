import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('caddie_gazebo')
    navigation_pkg = get_package_share_directory('caddie_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_slam = LaunchConfiguration('use_slam')
    use_rviz = LaunchConfiguration('use_rviz')
    use_perception = LaunchConfiguration('use_perception')
    use_vlm = LaunchConfiguration('use_vlm')
    use_voice = LaunchConfiguration('use_voice')
    use_llm = LaunchConfiguration('use_llm')
    use_velocity_limiter = LaunchConfiguration('use_velocity_limiter')
    use_unitree_sport_bridge = LaunchConfiguration('use_unitree_sport_bridge')
    use_leg_animation = LaunchConfiguration('use_leg_animation')
    detector_backend = LaunchConfiguration('detector_backend')
    yolo_model = LaunchConfiguration('yolo_model')
    detection_rate = LaunchConfiguration('detection_rate')
    publish_annotated_image = LaunchConfiguration('publish_annotated_image')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'sim.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'use_leg_animation': use_leg_animation,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_slam': use_slam,
            'use_nav2': use_nav2,
            'use_rviz': use_rviz,
        }.items(),
    )

    ball_detector = Node(
        package='caddie_perception',
        executable='golf_ball_detector',
        name='golf_ball_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'detector_backend': detector_backend,
            'yolo_model': yolo_model,
            'detection_rate': ParameterValue(detection_rate, value_type=float),
            'publish_annotated_image': ParameterValue(
                publish_annotated_image, value_type=bool),
        }],
        condition=IfCondition(use_perception),
    )

    vlm_context = Node(
        package='caddie_perception',
        executable='vlm_scene_context',
        name='vlm_scene_context',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_vlm),
    )

    voice_node = Node(
        package='caddie_interaction',
        executable='voice_command_node',
        name='voice_command_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_microphone': ParameterValue(use_voice, value_type=bool),
        }],
    )

    llm_node = Node(
        package='caddie_interaction',
        executable='llm_command_node',
        name='llm_command_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_llm),
    )

    autonomy_node = Node(
        package='caddie_core',
        executable='autonomous_caddie',
        name='autonomous_caddie',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    velocity_limiter = Node(
        package='caddie_control',
        executable='go2_velocity_limiter',
        name='go2_velocity_limiter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_velocity_limiter),
    )

    unitree_sport_bridge = Node(
        package='caddie_control',
        executable='unitree_sportmode_bridge',
        name='unitree_sportmode_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_unitree_sport_bridge),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulated time'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start Gazebo GUI'),
        DeclareLaunchArgument(
            'use_slam', default_value='true',
            description='Start SLAM Toolbox'),
        DeclareLaunchArgument(
            'use_nav2', default_value='true',
            description='Start Nav2 navigation servers'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz'),
        DeclareLaunchArgument(
            'use_perception', default_value='true',
            description='Start golf ball detector'),
        DeclareLaunchArgument(
            'use_vlm', default_value='true',
            description='Start VLM scene-context adapter'),
        DeclareLaunchArgument(
            'use_voice', default_value='false',
            description='Enable microphone input in voice node'),
        DeclareLaunchArgument(
            'use_llm', default_value='true',
            description='Start conversational command router'),
        DeclareLaunchArgument(
            'use_velocity_limiter', default_value='false',
            description='Start optional /cmd_vel_raw to /cmd_vel limiter'),
        DeclareLaunchArgument(
            'use_unitree_sport_bridge', default_value='false',
            description='Bridge /cmd_vel to official Unitree /api/sport/request messages'),
        DeclareLaunchArgument(
            'use_leg_animation', default_value='true',
            description='Enable visible, surface-aware Go2 leg gait animation'),
        DeclareLaunchArgument(
            'detector_backend', default_value='auto',
            description='auto, yolo, or opencv'),
        DeclareLaunchArgument(
            'yolo_model', default_value='',
            description='Path/name for a custom YOLO golf-ball model'),
        DeclareLaunchArgument(
            'detection_rate', default_value='5.0',
            description='Golf-ball detector inference rate in Hz'),
        DeclareLaunchArgument(
            'publish_annotated_image', default_value='false',
            description='Publish /caddie/perception/annotated_image'),
        sim_launch,
        navigation_launch,
        ball_detector,
        vlm_context,
        voice_node,
        llm_node,
        autonomy_node,
        velocity_limiter,
        unitree_sport_bridge,
    ])

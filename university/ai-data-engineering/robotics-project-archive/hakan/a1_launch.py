from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, ExecuteProcess, 
                          SetEnvironmentVariable, DeclareLaunchArgument, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    pkg_share = get_package_share_directory('project')
    gazebo_share = get_package_share_directory('gazebo_ros')
    turtlebot3_share = get_package_share_directory('turtlebot3_gazebo')
    
    # Environment variables
    env_vars = [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', 
            f"{os.path.join(pkg_share, 'models')}:{os.path.join(turtlebot3_share, 'models')}"),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'),
        SetEnvironmentVariable('AUDIODEV', 'null'),
        SetEnvironmentVariable('DISPLAY', os.environ.get('DISPLAY', ':0')),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
    ]

    # Robot state publisher with proper robot description
    robot_desc_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_waffle.urdf'
    )

    with open(robot_desc_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                arguments=[
                    '-entity', 'turtlebot3',
                    '-file', os.path.join(turtlebot3_share, 'models', 'turtlebot3_waffle', 'model.sdf'),
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', '0.01'
                ],
                output='screen'
            )
        ]
    )

    rviz_config = os.path.join(pkg_share, 'rviz', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    cleanup_gazebo = ExecuteProcess(
        cmd=['pkill', '-9', 'gzserver'],
        name='cleanup_gazebo',
        output='screen'
    )

    box_manager_node = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_share, 'scripts', 'box_manager.py')],
        name='box_manager',
        output='screen'
    )

    robot_controller_node = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_share, 'scripts', 'robot_controller.py')],
        name='robot_controller',
        output='screen'
    )

    box_detector_node = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_share, 'scripts', 'box_detector.py')],
        name='box_detector',
        output='screen'
    )

    return LaunchDescription(env_vars + [
        cleanup_gazebo,
        robot_state_publisher,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')]),
            launch_arguments={
                'world': os.path.join(pkg_share, 'worlds', 'warehouse.world'),
                'verbose': 'true',
                'gui': 'true'
            }.items()
        ),
        spawn_robot,
        rviz_node,
        box_manager_node,
        robot_controller_node,
        box_detector_node
    ]) 
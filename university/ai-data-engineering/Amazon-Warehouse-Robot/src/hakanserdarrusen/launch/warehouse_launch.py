#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource



TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']



def launch_setup(context, *args, **kwargs):

    #Argumments
    worldFileName = LaunchConfiguration('world', default='our_warehouse.world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    x_pose = LaunchConfiguration('x_pose', default='8.9')
    y_pose = LaunchConfiguration('y_pose', default='4.0')
    world_file_name_str = worldFileName.perform(context)
    #Packages
    pkg_share_dir = get_package_share_directory('hakanserdarrusen')
    pkg_prefix_dir = get_package_prefix('hakanserdarrusen')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir_robot = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    nav2_file_dir = get_package_share_directory('turtlebot3_navigation2')
    slam_pkg_dir = get_package_share_directory('slam_toolbox')
    #pkg_turtlebot3_cartographer = get_package_share_directory('turtlebot3_cartographer')
    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    #Paths
    world_file_path = os.path.join(pkg_share_dir, 'worlds', world_file_name_str)
    
    #Launch Gazebo Server and Client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    #Launch Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    spawn_turtlebot_cmd = TimerAction(
    period=3.0,  # Wait for 5 seconds before running the spawn
    actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
            }.items()
        )
    ]
)


    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(nav2_file_dir, 'param', param_file_name)
            }.items()
    )

    
    #Launch Cartographer
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_dir, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    '''
    #Launch slam
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    '''


    referee_node = Node(
        package='hakanserdarrusen',
        executable='tracker',
        name='tracker',
        output='screen',
    )

    return [gzserver_cmd, gzclient_cmd, robot_state_publisher, spawn_turtlebot_cmd,nav2,cartographer,referee_node]


def generate_launch_description():

    #Argumments Declaration
   
    #Launch Description Declaration
    ld = LaunchDescription()

    #Add Actions

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld

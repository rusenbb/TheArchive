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
    worldFileName = "warehouse5.world"
    map_file = "my_map.yaml"
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    x_pose = LaunchConfiguration('x_pose', default='7')
    y_pose = LaunchConfiguration('y_pose', default='0')
    #Packages
    pkg_share_dir = get_package_share_directory('assignment2')
    pkg_prefix_dir = get_package_prefix('assignment2')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    world_file_path = os.path.join(pkg_share_dir, 'worlds', worldFileName)
    map_path = os.path.join(pkg_share_dir, 'maps', map_file)
    
    rviz_config_dir = os.path.join(pkg_share_dir,
                                'rviz', 'navigation.rviz')
    
    
    
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


    nav2 = Node(package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename':map_path},
                        {'topic_name':'map'},
                        {'frame_id':'map'}, 
                       ])

    amcl = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'initial_pose':{'x':0.0,'y':0.0,'z':0.0,'yaw':0.01}},
            {'set_initial_pose': True},
            {'first_map_only': True},
            {'use_sim_time':use_sim_time}])
    
    lifecycle = Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['amcl','map_server']}])
    
    # #Launch RViz
    # rviz =  Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz2',
    #         arguments=['-d', rviz_config_dir,
    #                    '--ros-args','--remap', 'use_sim_time:=true'],
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         output='log')
    
    #Launch referee
    referee_node = Node(
        package='assignment2',
        executable='referee',
        name='referee',
        output='screen',
    )

    return [gzserver_cmd, gzclient_cmd, robot_state_publisher, spawn_turtlebot_cmd, nav2, amcl, lifecycle, referee_node] #rviz


def generate_launch_description():

    #Argumments Declaration
   
    #Launch Description Declaration
    ld = LaunchDescription()

    #Add Actions

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld

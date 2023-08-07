import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)




def generate_launch_description():

    package_name='albot-description'

    pkg_path = os.path.join(get_package_share_directory('albot-description'))

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    world = LaunchConfiguration('world')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('aws_robomaker_small_warehouse_world'),
            'worlds',
            'no_roof_small_warehouse.sdf'
        ]),
        description='Full path to world model file to load'
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),

                launch_arguments={
                    'world': world}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'albot_robot', 
                                   '-z', '0.135'
                                   ],
                        output='screen')

    tag_height = 0.4 # Altura das tags

    atag01 = Node(package='gazebo_ros', executable='spawn_entity.py',  namespace='tag1', # I really need this?
                        arguments=['-database', 'Apriltag36_11_00001',
                                   '-entity', 'april_tag01', 
                                   '-x', '2.77',
                                   '-y', '0.55',
                                   '-z', f'{tag_height}',
                                   '-P', '-1.57',
                                   ],
                        output='screen')

    atag02 = Node(package='gazebo_ros', executable='spawn_entity.py',  namespace='tag2', # I really need this?
                        arguments=['-database', 'Apriltag36_11_00002',
                                   '-entity', 'april_tag02', 
                                   '-x', '2.77',
                                   '-y', '-1.25',
                                   '-z', f'{tag_height}',
                                   '-P', '-1.57',
                                   ],
                        output='screen')

    atag03 = Node(package='gazebo_ros', executable='spawn_entity.py', namespace='tag3', # I really need this?
                        arguments=['-database', 'Apriltag36_11_00003',
                                   '-entity', 'april_tag03', 
                                   '-x', '2.77',
                                   '-y', '-3',
                                   '-z', f'{tag_height}',
                                   '-P', '-1.57',
                                   ],
                        output='screen')

    atag04 = Node(package='gazebo_ros', executable='spawn_entity.py', namespace='tag4', # I really need this?
                            arguments=['-database', 'Apriltag36_11_00004',
                                    '-entity', 'april_tag04', 
                                    '-x', '2.77',
                                    '-y', '-4.84',
                                    '-z', f'{tag_height}',
                                    '-P', '-1.57',
                                    ],
                            output='screen')

    atag05 = Node(package='gazebo_ros', executable='spawn_entity.py', namespace='tag5', # I really need this?
                        arguments=['-database', 'Apriltag36_11_00005',
                                   '-entity', 'april_tag05', 
                                   '-x', '2.77',
                                   '-y', '-6.75',
                                   '-z', f'{tag_height}',
                                   '-P', '-1.57',
                                   ],
                        output='screen')

    atag06 = Node(package='gazebo_ros', executable='spawn_entity.py',  namespace='tag6', # I really need this?
                            arguments=['-database', 'Apriltag36_11_00006',
                                    '-entity', 'april_tag06', 
                                    '-x', '2.77',
                                    '-y', '-8.66',
                                    '-z', f'{tag_height}',
                                    '-P', '-1.57',
                                    ],
                            output='screen')

    atag07 = Node(package='gazebo_ros', executable='spawn_entity.py',  namespace='tag7', # I really need this?
                            arguments=['-database', 'Apriltag36_11_00007',
                                    '-entity', 'april_tag07', 
                                    '-x', '1.8',
                                    '-y', '-10.31',
                                    '-z', f'{tag_height}',
                                    '-P', '-1.57',
                                    '-R', '-1.57',
                                    ],
                            output='screen')


    # Launch them all!
    return LaunchDescription([
        rsp,
        declare_world_cmd,
        gazebo,
        atag01,
        atag02,
        atag03,
        atag04,
        atag05,
        atag06,
        atag07,       
        RegisterEventHandler(
        OnProcessExit(
                target_action=atag07,
                on_exit=[
                    LogInfo(msg='Spawning albot'),
                    spawn_entity
                ]
            )
        ),

        
    ])
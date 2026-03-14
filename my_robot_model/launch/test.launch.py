#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    pkg_path = get_package_share_directory('my_robot_model')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_path, 'rviz', 'config.rviz')

    robots = ['robot1', 'robot2']

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)

    for r in robots:

        robot_description = Command([
            'xacro ',
            xacro_file,
            ' prefix:=', r + '_'
        ])

        ld.add_action(
            GroupAction([
                PushRosNamespace(r),

                # ===== ROBOT STATE PUBLISHER =====
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    parameters=[{
                        'robot_description': robot_description,
                        'use_sim_time': False
                    }],
                    output='screen'
                ),


                # ===== WHEEL ODOM PUBLISHER (FIX HERE) =====
                Node(
                    package='my_robot_model',
                    executable='wheel_odom_publisher.py',
                    name='wheel_odom_publisher',
                    parameters=[
                        {'robot_name': r},
                        {'use_sim_time': False}
                    ],
                    output='screen'
                ),
            ])
        )

    # ===== RVIZ (CHỈ 1 INSTANCE) =====
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            parameters=[{'use_sim_time': False}],
            output='screen'
        )
    )

    return ld

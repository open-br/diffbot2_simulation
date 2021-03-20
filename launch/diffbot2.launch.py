#!/usr/bin/env python3

# Copyright 2020 Open Rise Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            name='namespace',
            default_value='diffbot2',
            description='Node namespace'
        ),
        DeclareLaunchArgument(
            name='robot_name',
            default_value=LaunchConfiguration('namespace'),
            description='Robot Name'
        ),
        DeclareLaunchArgument(
            name='server_only',
            default_value='false',
            description='No gui, only server'
        ),

        # ign gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                __path([FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'])
            ),
            launch_arguments={
                'ign_args': [
                    '-r ',
                    PythonExpression([
                        '"-s " if "true" == "', LaunchConfiguration('server_only'),
                        '" else ""'
                    ]),
                    __path([
                        FindPackageShare('diffbot2_simulation'), 'worlds', 'diffbot2_default.sdf'
                    ])
                ]
            }.items()
        ),

        # diffbot2_description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                __path([
                    FindPackageShare('diffbot2_description'), 'launch', 'spawn_robot.launch.py'
                ])
            ),
            launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
        ),

        # ros ign bridge
        Node(
            package='ros_ign_bridge',
            namespace=LaunchConfiguration('namespace'),
            executable='parameter_bridge',
            arguments=[
                ['/model/', LaunchConfiguration('namespace'),
                 '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],

                ['/model/', LaunchConfiguration('namespace'),
                 '/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry']
            ],
            output='screen',
            remappings=[
                (['/model/', LaunchConfiguration('namespace'), '/cmd_vel'],
                 ['/', LaunchConfiguration('namespace'), '/cmd_vel']),

                (['/model/', LaunchConfiguration('namespace'), '/odometry'],
                 ['/', LaunchConfiguration('namespace'), '/odometry'])]
        ),

        # robot spawn
        Node(
            package='ros_ign_gazebo',
            namespace=LaunchConfiguration('namespace'),
            executable='create',
            arguments=[
                '-world', 'default', '-string',
                __xacro_load([FindPackageShare('diffbot2_description'), 'urdf', 'diffbot2.xacro']),
                '-name',
                LaunchConfiguration('robot_name'), '-allow_renaming', 'true', '-z', '1'
            ],
            output='screen'
        ),
    ])


def __path(subst_array):
    return PathJoinSubstitution(subst_array)


def __xacro_load(xacro_path):
    return Command(['xacro ', __path(xacro_path)])

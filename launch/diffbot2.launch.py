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
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_params = LaunchDescription()

    launch_params.add_action(
        DeclareLaunchArgument(
            name='namespace', default_value='diffbot2',
            description='Node namespace'
        )
    )
    launch_params.add_action(
        DeclareLaunchArgument(
            name='robot_name', default_value=LaunchConfiguration('namespace'),
            description='Robot Name'
        )
    )

    launch_params.add_action(
        DeclareLaunchArgument(
            name='server_only', default_value='False',
            description='No gui, only server'
        )
    )

    pkg_diffbot2_desc_path_sub = FindPackageShare('diffbot2_description')
    pkg_diffbot2_simulation_path_sub = FindPackageShare('diffbot2_simulation')
    pkg_ros_ign_gazebo_path_sub = FindPackageShare('ros_ign_gazebo')

    should_launch_server_sub = PythonExpression([" ' -s ' ", ' if ',
                                                LaunchConfiguration('server_only'), ' else ',
                                                " ' ' "])

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_ros_ign_gazebo_path_sub, 'launch',
                 'ign_gazebo.launch.py'])),
        launch_arguments={
            'ign_args': ['-r ', should_launch_server_sub, PathJoinSubstitution(
                [pkg_diffbot2_simulation_path_sub,
                 'worlds', 'diffbot2_default.sdf'])]
        }.items())

    include_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([pkg_diffbot2_desc_path_sub,
                              'launch', 'spawn_robot.launch.py'])),
        launch_arguments={'namespace': LaunchConfiguration(
            'namespace')}.items())

    # There is no parameter server, each node has its own parameters

    xacro_command_sub = Command([
                                'xacro ',
                                PathJoinSubstitution([
                                            pkg_diffbot2_desc_path_sub,
                                            'urdf', 'diffbot2.xacro']),
                                ])

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[['/model/', LaunchConfiguration('namespace'),
                    '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
                   ['/model/', LaunchConfiguration('namespace'),
                    '/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry']],
        output='screen',
        remappings=[
                (['/model/', LaunchConfiguration('namespace'),
                  '/cmd_vel'],
                 ['/', LaunchConfiguration('namespace'),
                  '/cmd_vel']),
                (['/model/', LaunchConfiguration('namespace'),
                  '/odometry'],
                 ['/', LaunchConfiguration('namespace'),
                  '/odometry'])
                ],
        namespace=LaunchConfiguration('namespace')
    )

    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-world', 'default',
                   '-string', xacro_command_sub,
                   '-name', LaunchConfiguration('robot_name'),
                   '-allow_renaming', 'true',
                   '-z', '1'],
        output='screen',
        namespace=LaunchConfiguration('namespace')
    )

    return LaunchDescription([
        launch_params,
        include_launch,
        bridge,
        ign_gazebo,
        spawn_robot
        ])

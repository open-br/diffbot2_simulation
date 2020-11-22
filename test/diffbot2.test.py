# Copyright 2020 Open BR
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from rclpy import init
from rclpy.node import Node


def generate_test_description():
    share_dir = FindPackageShare('diffbot2_simulation')
    simulation_launch_path = PathJoinSubstitution([share_dir, 'launch', 'diffbot2.launch.py'])
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch_path),
        launch_arguments={'namespace': 'ns', 'server_only': 'True'}.items(),
    )

    return (
        LaunchDescription([
            simulation_launch,
            # Start tests right away - no need to wait for anything in this example.
            # In a more complicated launch description, we might want this action happen
            # once some process starts or once some other event happens
            TimerAction(
                    period=3.0,
                    actions=[ReadyToTest()]
                ),
        ])
    )


class TestSpawnLaunchInterface(unittest.TestCase):

    def test_topics(self):
        init()
        node = Node('me')
        topics = node.get_topic_names_and_types()
        node_timer = node.get_clock()
        time_start = node_timer.now()
        logger = node.get_logger()
        while (('/ns/cmd_vel', ['geometry_msgs/msg/Twist']) not in topics):
            logger.info(f'Timer: {(node_timer.now() - time_start)}, topics {topics} ')
            topics = node.get_topic_names_and_types()
            time.sleep(0.5)
        logger.info(f'Here!!!!!!!!!!! topics: {topics} ')
        self.assertIn(('/ns/joint_states', ['sensor_msgs/msg/JointState']), topics)
        self.assertIn(('/ns/robot_description', ['std_msgs/msg/String']), topics)
        self.assertIn(('/ns/cmd_vel', ['geometry_msgs/msg/Twist']), topics)
        self.assertIn(('/ns/odometry', ['nav_msgs/msg/Odometry']), topics)
        self.assertIn(('/tf', ['tf2_msgs/msg/TFMessage']), topics)
        self.assertIn(('/tf_static', ['tf2_msgs/msg/TFMessage']), topics)

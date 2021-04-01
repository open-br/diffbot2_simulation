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

from functools import reduce
import time
import unittest

from geometry_msgs.msg import Twist, Vector3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node as LaunchNode
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from nav_msgs.msg import Odometry

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def generate_test_description():
    diffbot2_launch = \
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('diffbot2_simulation'), 'launch', 'diffbot2.launch.py']),
            )

    return (
        LaunchDescription([
            IncludeLaunchDescription(
                diffbot2_launch,
                launch_arguments={'namespace': 'ns', 'server_only': 'True'}.items()),
            ReadyToTest(),
        ]),

        # Test Args
        {'diffbot2_launch': diffbot2_launch}
    )


class SubscriberTool():

    def __init__(self, node: Node = None):
        self.__this_node = node if node is not None else Node('internal_node')
        self.__this_executor = MultiThreadedExecutor()
        self.__this_executor.add_node(self.__this_node)
        self.__msg = {}

    def create_subscription(self, topic_name, msg_type):
        self.__msg[topic_name] = None

        def callback(msg):
            self.__this_node.get_logger().debug(
                f'called callback {topic_name}', throttle_duration_sec=0.1)

            self.__msg[topic_name] = msg

        self.__this_node.create_subscription(
            msg_type,
            topic_name,
            callback,
            10)

    def await_for_msg(self, topic: str, timeout: float = None):
        self.__msg[topic] = None
        task = self.__this_executor.create_task(self.is_msg, topic)
        self.__this_executor.spin_until_future_complete(task, timeout)

        self.__this_node.get_logger().debug(f'is taks done? {task.done()}')
        self.__this_node.get_logger().debug(f'task result: {task.result().twist.twist}')

        return self.__msg[topic]

    async def is_msg(self, topic):
        rclpy.spin_once(self.__this_node)  # force spin
        while self.__msg[topic] is None:
            continue
        return self.__msg[topic]


class TestSpawnLaunchInterface(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.this_node = Node('test_node')
        cls.cmd_vel_pub = cls.this_node.create_publisher(Twist, '/ns/cmd_vel', 1)
        cls.sub_tool = SubscriberTool()
        cls.sub_tool.create_subscription('/ns/odometry', Odometry)

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def test_nodes_startup(self, proc_info, diffbot2_launch: PythonLaunchDescriptionSource):
        entities = diffbot2_launch.try_get_launch_description_without_context().entities
        nodes = [entity for entity in entities if isinstance(entity, LaunchNode)]
        for node in nodes:
            proc_info.assertWaitForStartup(node, timeout=5)

    def test_topics(self):
        expected = [
            ('/ns/joint_states', ['sensor_msgs/msg/JointState']),
            ('/ns/robot_description', ['std_msgs/msg/String']),
            ('/ns/cmd_vel', ['geometry_msgs/msg/Twist']),
            ('/ns/odometry', ['nav_msgs/msg/Odometry']),
            ('/tf', ['tf2_msgs/msg/TFMessage']),
            ('/tf_static', ['tf2_msgs/msg/TFMessage'])
        ]
        topics = self.__get_topic_names_and_types(expected, timeout=0.5)
        self.assertIsNotNone(topics)

        for expected_topic in expected:
            self.assertIn(expected_topic, topics)

    def __get_topic_names_and_types(self, expected, timeout):
        """Make sure discovery has found all 'expected' topics."""
        start = time.monotonic()
        while True:
            topics = self.this_node.get_topic_names_and_types()
            now = time.monotonic()
            if all(expected_topic in topics for expected_topic in expected):
                return topics
            elif (now - start) > timeout:
                return None

    def test_robot_move(self):
        def assert_velocity(current, expected):
            self.assertIsNotNone(current)
            self.assertAlmostEqual(current.linear.x, expected.linear.x)
            self.assertAlmostEqual(current.angular.z, expected.angular.z)

        current_vel = self.__wait_for_vel(timeout_sec=5)
        assert_velocity(current_vel, Twist())

        vel_cmd = Twist(linear=Vector3(x=0.1))
        self.cmd_vel_pub.publish(vel_cmd)
        current_vel = self.__wait_for_vel(expected_vel=vel_cmd, timeout_sec=5)
        assert_velocity(current_vel, vel_cmd)

        vel_cmd = Twist(angular=Vector3(z=0.1))
        self.cmd_vel_pub.publish(vel_cmd)
        current_vel = self.__wait_for_vel(expected_vel=vel_cmd, timeout_sec=10)
        assert_velocity(current_vel, vel_cmd)

        vel_cmd = Twist()
        self.cmd_vel_pub.publish(vel_cmd)
        current_vel = self.__wait_for_vel(expected_vel=vel_cmd, timeout_sec=5)
        assert_velocity(current_vel, vel_cmd)

    def __wait_for_vel(self, expected_vel=None, timeout_sec=0):
        start = time.monotonic()
        end = start + timeout_sec
        fields = 'twist.twist'

        def get_current_vel():
            odom_msg = self.sub_tool.await_for_msg('/ns/odometry', timeout_sec)

            if odom_msg is None:
                return None
            else:
                return reduce(getattr, fields.split('.'), odom_msg)

        if expected_vel is None:
            return get_current_vel()

        else:
            while True:
                current_vel = get_current_vel()
                if current_vel is None:
                    return None

                linear_diff = abs(expected_vel.linear.x - current_vel.linear.x)
                angular_diff = abs(expected_vel.angular.z - current_vel.angular.z)
                now = time.monotonic()
                if (
                    (linear_diff < 0.00001 and angular_diff < 0.00001) or
                    now >= end
                ):
                    return current_vel

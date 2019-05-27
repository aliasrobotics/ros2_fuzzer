"""
ROS Fuzzer example test cases module.

:authors: Alias Robotics S.L. Borja Erice, Odei Olalde, Xabi Perez, Gorka Olalde
"""
import unittest
import rclpy
from rclpy.node import Node
from hypothesis import given, settings, Verbosity
from numpy import float64
from hypothesis.strategies import text
from rcl_interfaces.msg import ParameterEvent
from builtin_interfaces.msg import Time

from process_handling import FuzzedLocalProcessHandler
from ros_basic_strategies import array
from ros_commons import map_ros_types


class TestRosLogMessages(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = Node('fuzzer_node')
        self.pub = self.node.create_publisher(ParameterEvent, '/listener')
        self.process_handler = FuzzedLocalProcessHandler('/example_node')

    def tearDown(self):
        rclpy.shutdown('Shutting down fuzzer node')

    @given(time=map_ros_types(Time))
    def test_fuzz_parameter_event_message(self, time):
        parameter_event = ParameterEvent()
        parameter_event.stamp = time
        self.pub.publish(parameter_event)

    @given(parameter_event=map_ros_types(ParameterEvent))
    def test_fuzz_parameter_event_message_exclude(self, parameter_event):
        parameter_event.node = 'Fixed node name'
        self.pub.publish(parameter_event)

    @given(parameter_event=map_ros_types(ParameterEvent), node_name=text(min_size=10, max_size=20))
    def test_fuzz_parameter_event_message_arbitrary(self, parameter_event, node_name):
        parameter_event.node = node_name
        self.pub.publish(parameter_event)

    '''
    @settings(max_examples=5000, verbosity=Verbosity.verbose)
    @given(array(elements=float64(), min_size=6, max_size=6))
    def test_fuzz_message_jointstate_effort(self, fuzzed_fields):
        joint_state_message = JointState()
        joint_state_message.effort = fuzzed_fields
        self.pub.publish(joint_state_message)
        assert self.process_handler.check_if_alive() is True

    @settings(max_examples=5000, verbosity=Verbosity.verbose)
    @given(array(elements=float64(), min_size=6, max_size=6),
           array(elements=float64(), min_size=6, max_size=6),
           array(elements=float64(), min_size=6, max_size=6))
    def test_fuzz_message_jointstate_all(self, positions, velocities, efforts):
        joint_state_message = JointState()
        joint_state_message.position = positions
        joint_state_message.velocity = velocities
        joint_state_message.effort = efforts
        self.pub.publish(joint_state_message)
        assert self.process_handler.check_if_alive() is True
    '''
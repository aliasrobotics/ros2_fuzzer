import hypothesis.strategies as st
import re
import numpy as np
import hypothesis.extra.numpy as npst
import array
from ros_basic_strategies import array, string, time, duration
try:
    import rclpy
    from rclpy.node import Node

    from rosgraph_msgs.msg import *
    from geometry_msgs.msg import *
    from std_msgs.msg import *
    from sensor_msgs.msg import *
    from diagnostic_msgs.msg import *
    from nav_msgs.msg import *
    from shape_msgs.msg import *
    from stereo_msgs.msg import *
    from trajectory_msgs.msg import *
    from visualization_msgs.msg import *
    from builtin_interfaces.msg import *
    from lifecycle_msgs.msg import *
    from action_msgs.msg import *
    from unique_identifier_msgs.msg import *
except ImportError:
    print("Please install ROS 2 first")


class Fuzzer(Node):
    def __init__(self, topic, msg_type):
        rclpy.init()
        super().__init__('fuzzer')
        self.topic = topic
        self.msg_type = msg_type
        self.pub = self.create_publisher(msg_type, topic)

    def publish(self, msg):
        self.pub.publish(msg)

    def destroy_node(self):
        self.pub.destroy()
        rclpy.shutdown()


def map_ros_types(type_name):
    strategy_dict = {}
    slot_dict = type_name.get_fields_and_field_types()
    for s_name, s_type in slot_dict.items():
        try:
            # TODO Add time, duration, regex ^sequence\<(?P<type>(?P<complex>(?P<package>\w+)\/)(?P<basic>\w+))\>$, simple array with size
            if 'sequence' in s_type:
                type_regexp = re.search('\<([\w\/\_]+)\>', s_type)
                aux = type_regexp.group(1)
                if aux == 'string':
                    strategy_dict[s_name] = array(elements=string())
                else:
                    strategy_dict[s_name] = array(elements=npst.from_dtype(np.dtype(aux)))
            elif s_type is 'string':
                strategy_dict[s_name] = st.text()
            elif s_type is 'time':
                strategy_dict[s_name] = time()
            elif s_type is 'duration':
                strategy_dict[s_name] = duration()
            else:  # numpy compatible ROS built-in types
                strategy_dict[s_name] = npst.from_dtype(np.dtype(s_type))
        except TypeError:
            # TODO: Complex type arrays
            if '/' in s_type and 'sequence' not in s_type:
                s_type_fix = s_type.split('/')[1]  # e.g. std_msgs/Header take just Header
                strategy_dict[s_name] = map_ros_types(eval(s_type_fix))
            elif '/' in s_type and 'sequence' in s_type:
                type_regexp = re.search('\<([\w\/\_]+)\>', s_type)
                aux = type_regexp.group(1)
                s_type_fix = aux.split('/')[1]
                strategy_dict[s_name] = array(elements=map_ros_types(eval(s_type_fix)))
    return dynamic_strategy_generator_ros(type_name, strategy_dict)


# A better approach. It returns an instance of a ROS msg directly, so no need for mapping! :)
@st.composite
def dynamic_strategy_generator_ros(draw, type_name, strategy_dict):  # This generates existing ROS msgs objects
    aux_obj = type_name()
    for key, value in strategy_dict.items():
        x = draw(value)
        print(type(x))
        if isinstance(x, list):
            print(x)
        # if it is numpy type, convert to python basic type
        if hasattr(x, 'dtype'):
            x = x.item()
        setattr(aux_obj, key, x)
    return aux_obj

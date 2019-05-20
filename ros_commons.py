import hypothesis.strategies as st
import numpy as np
import hypothesis.extra.numpy as npst
from ros_generic_fuzzer.ros_basic_strategies import array, string, time, duration
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
except ImportError:
    print("Please install ROS 2 first")


class Fuzzer(Node):
    def __init__(self, topic, msg_type):
        super().__init__('fuzzer')
        self.topic = topic
        self.msg_type = msg_type
        self.pub = self.create_publisher(msg_type, topic)

    def publish(self, msg):
        self.publish(msg)

    def destroy_node(self):
        self.destroy_node()
        rclpy.shutdown()


def map_ros_types(type_name):
    strategy_dict = {}
    slot_dict = type_name.get_fields_and_field_types()
    for s_name, s_type in slot_dict.items():
        try:
            # TODO Add time, duration
            if '[' and ']' in s_type:
                array_size = s_type[s_type.index('[') + 1:s_type.index(']')]
                if array_size == '':
                    array_size = None  # TODO: not None!
                else:
                    array_size = int(array_size)
                aux = s_type.split('[')[0]
                if aux == 'string':
                    strategy_dict[s_name] = array(elements=string(), min_size=array_size, max_size=array_size)
                else:
                    strategy_dict[s_name] = array(elements=npst.from_dtype(np.dtype(aux)), min_size=array_size,
                                                  max_size=array_size)
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
            if '/' in s_type and '[]' not in s_type:
                s_type_fix = s_type.split('/')[1]  # e.g. std_msgs/Header take just Header
                strategy_dict[s_name] = map_ros_types(eval(s_type_fix))
            elif '/' in s_type and '[]' in s_type:
                # TODO: Implement complex types fixed value arrays
                s_type_fix = s_type.split('/')[1].split('[')[0]  # e.g. std_msgs/Header take just Header
                strategy_dict[s_name] = array(elements=map_ros_types(eval(s_type_fix)))
    return dynamic_strategy_generator_ros(type_name, strategy_dict)


# A better approach. It returns an instance of a ROS msg directly, so no need for mapping! :)
@st.composite
def dynamic_strategy_generator_ros(draw, type_name, strategy_dict):  # This generates existing ROS msgs objects
    aux_obj = type_name()
    for key, value in strategy_dict.items():
        setattr(aux_obj, key, draw(value))
    return aux_obj

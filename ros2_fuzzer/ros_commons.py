"""
ROS Fuzzer base module.

:authors: Alias Robotics S.L. Borja Erice, Odei Olalde, Xabi Perez, Gorka Olalde
"""
import importlib
import re
import numpy as np
import hypothesis.extra.numpy as npst
import hypothesis.strategies as st
import rclpy
from rclpy.node import Node
from ros2_fuzzer.ros_basic_strategies import array, string


class ROS2NodeFuzzer(Node):
    """
    Class Helper to create ROS2 Nodes and execute common actions
    Defined context manager to start and kill the node when finished with its execution
    """
    def __init__(self):
        self.pub = None
        self.client = None

    def __enter__(self):
        rclpy.init()
        super().__init__('ROS2_Fuzzer')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.pub:
            self.pub.destroy()
        if self.client:
            self.client.destroy()
        rclpy.shutdown()

    def publish(self, msg_type, msg, topic):
        if self.pub:
            self.pub.publish(msg)
        else:
            self.pub = self.create_publisher(msg_type, topic)
            self.publish(msg_type, msg, topic)

    def send_request(self, srv_type, srv_name, srv_request):
        if self.client:
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            future = self.client.call_async(srv_request)
            rclpy.spin_until_future_complete(self, future)
        else:
            self.client = self.create_client(srv_type, srv_name)
            self.send_request(srv_type, srv_name, srv_request)


def ros_type_to_dict(msg_type):
    """
    Create a dictionary which values say if the ROS2 message type is complex (not basic), which is its parent
    ROS message module, its type, if it is an array and if so, its size.

    :param msg_type: ROS2 message type.
    :return: A dictionary which values say if the ROS2 message type is complex (not basic), which is its parent
             ROS2 message module, its type, if it is an array and if so, its size.
    """
    type_regexp = re.compile(
        r'^((?P<sequence>[\w]+)\<|(?P<complex>(?P<module>[\w]+)/)?(?P<type>[\w]+)|(?P<array>\[(?P<array_size>[0-9]*)?\])?|\>)+$')
    type_match = type_regexp.match(msg_type)
    if type_match:
        return type_match.groupdict()
    else:
        return None


# TODO possible duplicated functions ros_msg_loader and ros_srv_loader (srv loader called once,
#  msg_loader called more than once)
def ros_msg_loader(type_dict):
    """
    Dynamically import ROS2 message modules.

    :param type_dict: A dictionary which values say if the ROS message type is complex (not basic), which is its parent
                      ROS2 message module, its type, if it is an array and if so, its size.
    :return: The ROS2 message class. If the provided type does not exist, raises an import error.
    """
    try:
        module = importlib.import_module(type_dict['module'] + '.msg')
        return module.__dict__[type_dict['type']]
    except KeyError:
        raise KeyError('ROS2 message type: {} not included in message module: {}'.format(type_dict['type'],
                                                                                         type_dict['module']))
    except ImportError:
        raise ImportError('ROS2 message module: {} does not exist.'.format(type_dict['module']))
    except TypeError:
        raise TypeError('ROS2 message type: {} does not exist'.format(type_dict['type']))


def ros_srv_loader(type_dict):
    """
    Dynamically import ROS2 service modules.

    :param type_dict: A dictionary which values say if the ROS service type is complex (not basic), which is its parent
                      ROS2 service module, its type, if it is an array and if so, its size.
    :return: The ROS2 service class. If the provided type does not exist, raises an import error.
    """
    try:
        module = importlib.import_module(type_dict['module'] + '.srv')
        return module.__dict__[type_dict['type']]
    except KeyError:
        raise KeyError('ROS2 service type: {} not included in service module: {}'.format(type_dict['type'],
                                                                                         type_dict['module']))
    except ImportError:
        raise ImportError('ROS2 service module: {} does not exist.'.format(type_dict['module']))
    except TypeError:
        raise TypeError('ROS2 service type: {} does not exist'.format(type_dict['type']))


def ros_interface_loader_str(ros2_type, interface_type):
    """
    Wrapper for the :func:`ros_msg_loader` to treat string type command line arguments.

    :param interface_type: A string representing the interface type (message, service,...)
    :param ros2_type: A string type ROS2 interface type (e.g. "rosgraph_msgs/Log").
    :return: The :func:`ros_srv_loader` or :func:`ros_msg_loader` function.
    """
    type_dict = ros_type_to_dict(ros2_type)
    if type_dict:
        if interface_type == 'service':
            return ros_srv_loader(type_dict)
        else:
            return ros_msg_loader(type_dict)
    else:
        raise ImportError('Unable to find defined ROS2 Interface type: {}'.format(ros2_type))


def map_ros_types(ros_class):
    """
        A recursive function that maps ROS2 message fields to Hypothesis strategies.

        :param ros_class: The ROS2 class to be fuzzed.
        :return: A function that generates Hypothesis strategies for a given ROS message type.
        """
    strategy_dict = {}
    slot_dict = ros_class.get_fields_and_field_types()
    for s_name, s_type in slot_dict.items():
        type_dict = ros_type_to_dict(s_type)
        if type_dict:
            if not type_dict['complex']:
                if type_dict['array'] or type_dict['sequence']:
                    parse_basic_arrays(s_name, type_dict, strategy_dict)
                elif type_dict['type'] == 'string':
                    strategy_dict[s_name] = st.text()
                elif type_dict['type'] == 'boolean':
                    strategy_dict[s_name] = st.booleans()
                elif type_dict['type'] == 'octet':
                    strategy_dict[s_name] = st.binary(min_size=1, max_size=1)
                else:  # numpy compatible ROS built-in types
                    strategy_dict[s_name] = npst.from_dtype(np.dtype(type_dict['type']))
            else:
                parse_complex_types(s_name, type_dict, strategy_dict)
    return dynamic_strategy_generator_ros(ros_class, strategy_dict)


def parse_basic_arrays(s_name, type_dict, strategy_dict):
    """
    Generate Hypothesis strategies for array types.

    :param s_name: Slot name to be parsed.
    :param type_dict: A dictionary which values say if the ROS2 message type is complex (not basic), which is its parent
                      ROS2 message module, its type, if it is an array and if so, its size.
    :param strategy_dict: A pointer to a dictionary to be filled with Hypothesis strategies.
    """
    if type_dict['array_size']:
        array_size = int(type_dict['array_size'])
    else:
        array_size = None
    if type_dict['type'] == 'string':
        strategy_dict[s_name] = array(elements=string(), min_size=array_size, max_size=array_size)
    elif type_dict['type'] == 'boolean':
        strategy_dict[s_name] = array(elements=npst.from_dtype(np.dtype('bool')), min_size=array_size,
                                      max_size=array_size)
    # TODO correct data type?
    elif type_dict['type'] == 'octet':
        strategy_dict[s_name] = array(elements=st.binary(min_size=1, max_size=1), min_size=array_size,
                                      max_size=array_size)
    else:
        strategy_dict[s_name] = array(elements=npst.from_dtype(np.dtype(type_dict['type'])), min_size=array_size,
                                      max_size=array_size)


def parse_complex_types(s_name, type_dict, strategy_dict):
    """
    Generate Hypothesis strategies for complex ROS2 types.

    :param s_name: Slot name to be parsed.
    :param type_dict: A dictionary which values say if the ROS2 message type is complex (not basic), which is its parent
                      ROS2 message module, its type, if it is an array or sequence and if so, its size.
    :param strategy_dict: A pointer to a dictionary to be filled with Hypothesis strategies.
    """
    if not type_dict['array'] and not type_dict['sequence']:
        strategy_dict[s_name] = map_ros_types(ros_msg_loader(type_dict))
    else:
        if type_dict['array_size']:
            strategy_dict[s_name] = array(elements=map_ros_types(ros_msg_loader(type_dict)),
                                          min_size=int(type_dict['array_size']),
                                          max_size=int(type_dict['array_size']))
        else:
            strategy_dict[s_name] = array(elements=map_ros_types(ros_msg_loader(type_dict)))


# A better approach. It returns an instance of a ROS2 msg directly, so no need for mapping! :)
@st.composite
def dynamic_strategy_generator_ros(draw, type_name, strategy_dict):  # This generates existing ROS msgs objects
    aux_obj = type_name()
    for key, value in strategy_dict.items():
        value = draw(value)
        # If it is array of numpy, get array python basic type
        if isinstance(value, list) and value and hasattr(value[0], 'dtype'):
            value = np.array(value)
            value = value.tolist()
        elif hasattr(value, 'dtype'):
            value = value.item()
        setattr(aux_obj, key, value)
    return aux_obj

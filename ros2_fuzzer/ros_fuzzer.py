from ros_commons import Fuzzer, map_ros_types
from hypothesis import given, settings, Verbosity
import logging
from argparse import ArgumentParser

# TODO delete or modify imports!!
from std_msgs.msg import *
from geometry_msgs.msg import *
from builtin_interfaces.msg import *
import rclpy

def test_main_wrapper(msg_type, topic):
    rclpy.init()
    fuzzer = Fuzzer(topic, msg_type)

    @settings(verbosity=Verbosity.verbose)
    @given(msg=map_ros_types(msg_type))
    def test_main(msg):
        fuzzer.publish(msg)

    test_main()
    fuzzer.destroy_node()
    rclpy.shutdown()


def main():
    """
    Main method
    """
    logging.basicConfig()
    logger = logging.getLogger(__name__)
    parser = ArgumentParser(description='ROS 2 subscriber fuzzer')
    parser.add_argument('-m', '--message', help='Message type to be fuzzed.', required=True)
    parser.add_argument('-t', '--topic', help='Topic name to be fuzzed.', required=True)
    args = parser.parse_args()
    if (args.message is None or args.topic is None) and args.length is None:
        parser.print_help()
    else:
        #try:
            #if check_msg_type(ros_msg_list(), args.message):
        test_main_wrapper(eval(args.message), args.topic)
            #else:
                #logger.warning('Invalid ROS 2 data type')
        #except Exception as e:
            #logger.critical('Exception occurred during execution --> ' + str(e))


if __name__ == '__main__':
    main()









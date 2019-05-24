import logging
from argparse import ArgumentParser
from hypothesis import given, settings, Verbosity
from ros2_fuzzer.ros_commons import Fuzzer, ros_msg_loader_str, map_ros_types


def test_main_wrapper(msg_type, topic):
    fuzzer = Fuzzer(topic, msg_type)
    @settings(max_examples=100, verbosity=Verbosity.verbose)
    @given(msg=map_ros_types(msg_type))
    def test_main(msg):
        fuzzer.publish(msg)
    test_main()
    fuzzer.destroy_node()


def main():
    """
    Main method. Takes two command line arguments, parses them and calls :func:`test_main_wrapper`
    """
    logging.basicConfig()
    logger = logging.getLogger(__name__)
    parser = ArgumentParser(description='ROS2 subscriber fuzzer')
    parser.add_argument('-m', '--message', help='Message type to be fuzzed.', required=True)
    parser.add_argument('-t', '--topic', help='Topic name to be fuzzed.', required=True)
    args = parser.parse_args()
    try:
        msg_type = ros_msg_loader_str(args.message)
        test_main_wrapper(msg_type, args.topic)
    except Exception as e:
        logger.critical('Exception occurred during execution --> ' + str(e))


if __name__ == '__main__':
    main()









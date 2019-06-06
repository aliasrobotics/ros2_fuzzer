import logging
from argparse import ArgumentParser
from hypothesis import given, settings, Verbosity, HealthCheck
from ros_commons import MessageFuzzer, ServiceFuzzer, ros_interface_loader_str, map_ros_types


def test_main_wrapper(msg_type, topic):
    fuzzer = MessageFuzzer(topic, msg_type)
    @settings(max_examples=100, verbosity=Verbosity.verbose)
    @given(msg=map_ros_types(msg_type))
    def test_main(msg):
        fuzzer.publish(msg)
    test_main()
    fuzzer.destroy_publisher()


def test_srv_fuzzing(srv_type, srv_type_request, srv_name):
    fuzzer = ServiceFuzzer(srv_type, srv_name)
    @settings(max_examples=100, verbosity=Verbosity.verbose, suppress_health_check=[HealthCheck.too_slow])
    @given(srv_request=map_ros_types(srv_type_request))
    def test_main(srv_request):
        fuzzer.send_request(srv_request)
    test_main()
    fuzzer.destroy_client()


def main():
    """
    Main method. Takes two command line arguments, parses them and calls :func:`test_main_wrapper`
    """
    logging.basicConfig()
    logger = logging.getLogger(__name__)
    parser = ArgumentParser(description='ROS2 Fuzzer', usage='''[-h] <interface_type> [<args>]

The possible interface types are:
    message
    service
    ''')
    subparsers = parser.add_subparsers(dest='interface_type')
    subparsers.required = True

    message_parser = subparsers.add_parser('message', help='Message command help')
    message_parser.add_argument('message_type', help='Message type to be fuzzed')
    message_parser.add_argument('topic', help='The topic to be fuzzed with the corresponding message_type')

    service_parser = subparsers.add_parser('service', help='Service command help')
    service_parser.add_argument('service_type', help='Service type to be fuzzed.')
    service_parser.add_argument('service_name', help='Service name to be fuzzed.')

    args = parser.parse_args()

    try:
        if args.interface_type == 'message':
            msg_type = ros_interface_loader_str(args.message_type, args.interface_type)
            test_main_wrapper(msg_type, args.topic)
        elif args.interface_type == 'service':
            srv_type = ros_interface_loader_str(args.service_type, args.interface_type)
            test_srv_fuzzing(srv_type, srv_type.Request, args.service_name)
    except Exception as e:
        logger.critical('Exception occurred during execution --> ' + str(e))


if __name__ == '__main__':
    main()









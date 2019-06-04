from argparse import ArgumentParser
import importlib
import rclpy
from hypothesis import given, settings, Verbosity
from ros_commons import map_ros_types


def get_service_names(node, srv_type):
    service_dict = dict(node.get_service_names_and_types())
    print(service_dict)
    service_name_list = []
    for service_name, service_type in service_dict.items():
        if service_type[0].rsplit('/', 1)[1] == srv_type:
            service_name_list.append(service_name.lstrip('/'))
    return service_name_list


def test_srv_fuzzing(srv_type, srv_type_request):
    rclpy.init()
    node = rclpy.create_node('fuzzer')
    @settings(max_examples=100, verbosity=Verbosity.verbose)
    @given(srv_request=map_ros_types(srv_type_request))
    def test_main(srv_request):
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service not available, waiting again...')
        future = client.call_async(srv_request)
        rclpy.spin_until_future_complete(node, future)
        print(f'future result: {future.result}')

    for service_name in get_service_names(node, srv_type):
        client = node.create_client(srv_type, service_name)
        test_main()

    node.destroy_node()
    rclpy.shutdown()


def main():
    parser = ArgumentParser(description='ROS2 service fuzzer')
    parser.add_argument('-s', '--service', help='Service type to be fuzzed.', required=True)
    parser.add_argument('-t', '--topic', help='Topic name to be fuzzed.', required=True)
    args = parser.parse_args()

    module = importlib.import_module(args.topic + '.srv')
    srv_type = module.__dict__[args.service]

    test_srv_fuzzing(srv_type, srv_type.Request)


if __name__ == '__main__':
    main()




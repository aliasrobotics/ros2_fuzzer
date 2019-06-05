from argparse import ArgumentParser
import importlib
import rclpy
from hypothesis import given, settings, Verbosity, HealthCheck
from ros_commons import map_ros_types


def test_srv_fuzzing(srv_type, srv_type_request, srv_name):
    rclpy.init()
    node = rclpy.create_node('fuzzer')
    client = node.create_client(srv_type, srv_name)
    @settings(max_examples=100, verbosity=Verbosity.verbose, suppress_health_check=[HealthCheck.too_slow])
    @given(srv_request=map_ros_types(srv_type_request))
    def test_main(srv_request):
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service not available, waiting again...')
        future = client.call_async(srv_request)
        rclpy.spin_until_future_complete(node, future)
    test_main()
    node.destroy_node()
    rclpy.shutdown()


def main():
    parser = ArgumentParser(description='ROS2 service fuzzer')
    parser.add_argument('-s', '--service_file', help='Service type to be fuzzed.', required=True)
    parser.add_argument('-st', '--service_type', help='Service type to be fuzzed.', required=True)
    parser.add_argument('-n', '--service_name', help='Service name to be fuzzed.', required=True)
    args = parser.parse_args()

    module = importlib.import_module(args.service_file + '.srv')
    srv_type = module.__dict__[args.service_type]

    test_srv_fuzzing(srv_type, srv_type.Request, args.service_name)


if __name__ == '__main__':
    main()




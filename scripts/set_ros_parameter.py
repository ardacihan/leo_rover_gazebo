#!/usr/bin/env python3
"""Set one ROS parameter without relying on the ros2cli daemon."""

import ast
import sys

import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters


def main():
    if len(sys.argv) != 4:
        raise SystemExit(
            'usage: set_ros_parameter.py NODE PARAMETER PYTHON_VALUE')
    target, name, encoded_value = sys.argv[1:]
    value = ast.literal_eval(encoded_value)
    rclpy.init()
    node = rclpy.create_node('validation_parameter_setter')
    service = target.rstrip('/') + '/set_parameters'
    client = node.create_client(SetParameters, service)
    if not client.wait_for_service(timeout_sec=10):
        raise SystemExit(f'parameter service unavailable: {target}')
    request = SetParameters.Request()
    request.parameters = [Parameter(name=name, value=value).to_parameter_msg()]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    response = future.result()
    results = response.results if response else None
    if not results or not results[0].successful:
        reason = results[0].reason if results else 'no response'
        raise SystemExit(f'parameter rejected: {reason}')
    print(f'{target} {name}={value}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

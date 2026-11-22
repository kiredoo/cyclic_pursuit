"""Client for the cyclic pursuit controller's per-agent landing services."""
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger


def load_cf_names_from_yaml(yaml_path):
    with open(yaml_path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    names = data['params']['ros__parameters']['cfs_active_names']
    if not isinstance(names, list) or not names:
        raise ValueError('cyclic.yaml must contain nonempty cfs_active_names')
    return names


def call_trigger(node, cf_name):
    service_name = f'/{cf_name}/trigger_landing'
    client = node.create_client(Trigger, service_name)
    while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f'Waiting for service {service_name}...')
    if not rclpy.ok():
        return None
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future)
    return future.result()


def main():
    config_file = Path(get_package_share_directory('cyclic_pursuit')) / 'config' / 'cyclic.yaml'
    active_names = load_cf_names_from_yaml(config_file)
    rclpy.init()
    node = rclpy.create_node('landing_client')
    try:
        for cf_name in active_names:
            response = call_trigger(node, cf_name)
            node.get_logger().info(
                f'{cf_name}: {response.message}' if response else f'{cf_name}: no response'
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

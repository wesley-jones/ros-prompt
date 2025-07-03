import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import json
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import os

def discover_actions(node):
    """Return a dict of {action_base_name: action_type} for available actions."""
    service_types = dict(node.get_service_names_and_types())
    actions = {}
    for service, types in service_types.items():
        if service.endswith('/_action/send_goal'):
            action_base = service[:-len('/_action/send_goal')]
            action_type = types[0]
            # Optionally clean up the type (strip _SendGoal)
            if action_type.endswith('_SendGoal'):
                action_type = action_type[:-len('_SendGoal')]
            actions[action_base] = action_type
    return actions

class CapabilityScannerNode(Node):
    def __init__(self):
        super().__init__('capability_scanner')
        # Load param
        self.declare_parameter('robot_caps', '')
        caps = self.get_parameter('robot_caps').get_parameter_value().string_value
        self.get_logger().info(f"Robot capabilities parameter: {caps}")

        if not caps:
            config_path = os.path.join(
                get_package_share_directory('ros_prompt'),
                'config',
                'robot_caps.yaml'
            )
            self.get_logger().info(f"Loading robot capabilities from default config: {config_path}")
            with open(config_path, 'r') as f:
                caps_dict = yaml.safe_load(f)
            
            self.get_logger().info(f"Loaded robot capabilities: {caps_dict}")
            caps = caps_dict['robot_caps'] if 'robot_caps' in caps_dict else caps_dict
            self.get_logger().info(f"Using robot capabilities: {caps}")
        else:
            # Try to parse if it's a string, else assume it's already a dict
            try:
                caps_dict = yaml.safe_load(caps)
                caps = caps_dict['robot_caps'] if 'robot_caps' in caps_dict else caps_dict
            except Exception:
                pass  # Already a dict, or not parseable as YAML
        self.robot_caps = caps

        # Discover live
        topics = dict(self.get_topic_names_and_types())
        self.get_logger().info(f"Discovered topics: {topics}")
        actions = discover_actions(self)
        self.get_logger().info(f"Discovered actions: {actions}")
        services = dict(self.get_service_names_and_types())
        self.get_logger().info(f"Discovered services: {services}")

        def filter_alive(caps, discovered):
            return [cap for cap in caps if cap['name'] in discovered]

        manifest = {
            "topics": filter_alive(self.robot_caps.get('topics', []), topics),
            "actions": filter_alive(self.robot_caps.get('actions', []), actions),
            "services": filter_alive(self.robot_caps.get('services', []), services),
        }
        self.get_logger().info(f"Filtered capabilities manifest: {manifest}")

        # Publish manifest
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(String, '/capabilities', qos_profile)
        self.pub.publish(String(data=json.dumps(manifest)))
        self.get_logger().info("Published robot capabilities manifest.")

def main(args=None):
    rclpy.init(args=args)
    node = CapabilityScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

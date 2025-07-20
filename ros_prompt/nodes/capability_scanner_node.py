import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import json
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import os
import importlib
import re
from ros_prompt.utilities.ros_type_utils import import_ros_type

# Define the mapping in one place
TYPENAME_MAP = {
    'double':  'float',
    'float':   'float',
    'float64': 'float',
    'float32': 'float',
    'int':     'int',
    'int64':   'int',
    'int32':   'int',
    'int16':   'int',
    'int8':    'int',
    'uint64':  'int',
    'uint32':  'int',
    'uint16':  'int',
    'uint8':   'int',
    'bool':    'bool',
    'boolean': 'bool',
    'string':  'str',
    'octet':   'int'
}
ROS_BUILTIN_MSGS = {
    'builtin_interfaces/Time',
    'builtin_interfaces/Duration',
    'std_msgs/Header',
}

BUILTIN_TYPES = set(TYPENAME_MAP.keys())
PRIMITIVE_TYPES_REGEX = '|'.join(re.escape(t) for t in TYPENAME_MAP.keys())
PRIMITIVE_REGEX = r'^(' + PRIMITIVE_TYPES_REGEX + r')\[(\d*)\]$'
SEQUENCE_REGEX = re.compile(rf'^sequence<({PRIMITIVE_TYPES_REGEX})>$')
# Matches sequence<TYPE>, captures TYPE
ANY_SEQUENCE_REGEX = re.compile(r'^sequence<(.+)>$')

def merge_params(dynamic_params: dict, caps_params: dict) -> dict:
    """
    For each parameter key found in robot_caps, replace the whole
    definition (type/size/value/whatever).  Keep everything else
    from the dynamic scan unchanged.
    """
    merged = dynamic_params.copy()
    merged.update(caps_params)          # caps wins on key conflicts
    return merged

def is_primitive_array(field_type):
    return re.match(PRIMITIVE_REGEX, field_type)

def extract_params_from_msg(self, msg_cls, prefix=''):
    params = {}
    for field, field_type in msg_cls._fields_and_field_types.items():
        # self.get_logger().info(f"Processing field: {field} of type {field_type}")
        # 1. Primitives
        if field_type in BUILTIN_TYPES:
            param_name = f"{prefix}{field}"
            params[param_name] = {'type': TYPENAME_MAP[field_type]}
        # 2. Arrays like double[9], float32[]
        elif is_primitive_array(field_type):
            m = is_primitive_array(field_type)
            base_type = m.group(1)
            size = m.group(2)
            param_name = f"{prefix}{field}"
            params[param_name] = {
                'type': f'List[{TYPENAME_MAP[base_type]}]',
                'size': int(size) if size else None
            }
        # 3. Sequences like sequence<string>
        elif SEQUENCE_REGEX.match(field_type):
            base_type = SEQUENCE_REGEX.match(field_type).group(1)
            param_name = f"{prefix}{field}"
            params[param_name] = {
                'type': f'List[{TYPENAME_MAP[base_type]}]',
                'size': None
            }
        # 4. ROS built-ins
        elif field_type in ROS_BUILTIN_MSGS:
            continue  # Skip ROS built-in types, no params needed
        # 5. Any sequence like sequence<TYPE>
        elif ANY_SEQUENCE_REGEX.match(field_type):
            base_type = ANY_SEQUENCE_REGEX.match(field_type).group(1)
            # Is it a primitive? (already handled above, but safe to check)
            if base_type in BUILTIN_TYPES:
                param_name = f"{prefix}{field}"
                params[param_name] = {
                    'type': f'List[{TYPENAME_MAP[base_type]}]',
                    'size': None
                }
            else:
                # It's a sequence of messages!
                # try:
                    # self.get_logger().info(f"Importing message class for sequence element: {base_type}")
                    nested_cls = import_ros_type(self, base_type)
                    # Recursively build the params for an item, then wrap in List[...] for manifest
                    nested_params = extract_params_from_msg(self, nested_cls, prefix=f"{prefix}{field}_item_")
                    params[f"{prefix}{field}"] = {
                        'type': f'List[{base_type}]',
                        'fields': nested_params
                    }
                # except Exception as e:
                #     self.get_logger().warning(f"Skipping sequence field {field} ({field_type}): {e}")
                #     continue
        # 6. Otherwise, assume it's a nested msg
        else:
            # try:
                # Nested type, recurse
                nested_cls = import_ros_type(self, field_type)
                params.update(extract_params_from_msg(self, nested_cls, prefix=f"{prefix}{field}_"))
            # except Exception as e:
            #     self.get_logger().info(f"Skipping field {field} ({field_type}): {e}")
    return params

def load_robot_capabilities(self):
    # Load param
    self.declare_parameter('robot_caps', '')
    caps = self.get_parameter('robot_caps').get_parameter_value().string_value
    # self.get_logger().info(f"Robot capabilities parameter: {caps}")

    if not caps:
        config_path = os.path.join(
            get_package_share_directory('ros_prompt'),
            'config',
            'robot_caps.yaml'
        )
        self.get_logger().info(f"Loading robot capabilities from default config: {config_path}")
        with open(config_path, 'r') as f:
            caps_dict = yaml.safe_load(f)

        # self.get_logger().info(f"Loaded robot capabilities: {caps_dict}")
        caps = caps_dict['robot_caps'] if 'robot_caps' in caps_dict else caps_dict
        # self.get_logger().info(f"Using robot capabilities: {caps}")
    else:
        # Try to parse if it's a string, else assume it's already a dict
        try:
            caps_dict = yaml.safe_load(caps)
            caps = caps_dict['robot_caps'] if 'robot_caps' in caps_dict else caps_dict
        except Exception:
            pass  # Already a dict, or not parseable as YAML
    return caps

def merge_manifest_with_capabilities(self, dynamic_manifest, robot_caps):
    """
    Merges actions and topics present in robot_caps, adds descriptions, and merges any extra metadata.
    :param dynamic_manifest: dict with 'topics' and 'actions'
    :param robot_caps: dict with 'topics', 'actions', and 'builtins'
    :return: merged manifest dict
    """
    def merge_section(dynamic_section, caps_section, section_name):
        """
        Keep ONLY the capabilities that are listed in robot_caps.yaml.
        Within each one, merge params with robot_caps taking precedence.
        """
        dyn_map = {d['name']: d for d in dynamic_section}

        merged = []
        for cap in caps_section:
            name = cap['name']

            # Filter – skip if the capability wasn’t discovered at runtime
            if name not in dyn_map:
                self.get_logger().info(
                    f"Skipping {section_name} '{name}' — in robot_caps.yaml "
                    f"but not found by scanner.")
                continue

            scanned = dyn_map[name]

            # Start with scanned, then overlay robot_caps
            merged_item = scanned.copy()
            for key, value in cap.items():
                if key == 'params' and isinstance(value, dict):
                    dyn_params = scanned.get('params', {})
                    merged_item['params'] = merge_params(dyn_params, value)
                elif key != 'name':
                    merged_item[key] = value

            merged.append(merged_item)

        return merged

    merged_topics = merge_section(dynamic_manifest.get('topics', []), robot_caps.get('topics', []), 'topic')
    merged_actions = merge_section(dynamic_manifest.get('actions', []), robot_caps.get('actions', []), 'action')
    merged_builtins = robot_caps.get('builtins', [])

    return {'topics': merged_topics, 'actions': merged_actions, 'builtins': merged_builtins}


def load_dynamic_manifest(self):
    manifest = {'topics': [], 'actions': []}

    # --- 1. Scan topics ---
    topic_types = self.get_topic_names_and_types()
    for topic_name, type_list in topic_types:
        # self.get_logger().info(f"Discovered topic: {topic_name} with types: {type_list}")
        # Some topics might have multiple types; pick the first
        msg_type_str = type_list[0]

        # Skip internal action message types
        if (
            msg_type_str.endswith('FeedbackMessage') or
            msg_type_str.endswith('Result') or
            msg_type_str.endswith('Goal')
        ):
            continue

        try:
            msg_cls = import_ros_type(self, msg_type_str)
        except Exception as e:
            self.get_logger().error(f"Failed to import message class for {msg_type_str}: {e}")
            continue
        params = extract_params_from_msg(self, msg_cls)
        entry = {
            'name': topic_name,
            'type': msg_type_str,
            'plugin_name': f"Set{topic_name.strip('/').replace('/', '_').capitalize()}",
            'params': params,
        }
        manifest['topics'].append(entry)

    # --- 2. Scan actions ---
    action_map = discover_actions(self)  # { '/navigate_to_pose': 'nav2_msgs/action/NavigateToPose' }
    for action_name, action_type in action_map.items():
        try:
            action_cls = import_ros_type(self, action_type)
        except Exception as e:
            self.get_logger().error(f"Failed to import action class for {action_type}: {e}")
            continue
        params = extract_params_from_msg(self, action_cls.Goal)  # Use the Goal submessage for params
        entry = {
            'name': action_name,
            'type': action_type,
            'plugin_name': f"{action_type.split('/')[-1]}",
            'params': params,
            'interface': 'action_client',
        }
        manifest['actions'].append(entry)

    return manifest


def discover_actions(node):
    """Return a dict of {action_base_name: action_type} for available actions."""
    service_types = dict(node.get_service_names_and_types())
    actions = {}
    for service, types in service_types.items():
        # ROS2 actions have a service named `/my_action/_action/send_goal`
        if service.endswith('/_action/send_goal'):
            action_base = service[:-len('/_action/send_goal')]  # e.g. /navigate_to_pose
            # Try to extract the real action type (e.g., nav2_msgs/action/NavigateToPose)
            # The type looks like 'nav2_msgs/action/NavigateToPose_SendGoal'
            action_type = types[0]
            if action_type.endswith('_SendGoal'):
                action_type = action_type[:-len('_SendGoal')]
            actions[action_base] = action_type
    return actions


class CapabilityScannerNode(Node):

    def __init__(self):
        super().__init__('capability_scanner')
        self.dynamic_manifest = load_dynamic_manifest(self)
        self.robot_caps = load_robot_capabilities(self)
        final_manifest = merge_manifest_with_capabilities(self, self.dynamic_manifest, self.robot_caps)

        # Publish manifest
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(String, '/capabilities', qos_profile)
        self.pub.publish(String(data=json.dumps(final_manifest)))
        self.get_logger().info(f"Published capabilities manifest: {final_manifest}")

def main(args=None):
    rclpy.init(args=args)
    node = CapabilityScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

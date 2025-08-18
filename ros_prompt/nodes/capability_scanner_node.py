import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import json
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import os
import re
from ros_prompt.utilities.ros_type_utils import import_ros_type
from ros_prompt.utilities.constants import CapabilityCategory

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

def _hydrate_entry(entry: dict, logger):
    """
    • Import the interface class referenced in entry["type"]
        - topics   → msg class
        - actions  → <Action>.Goal
        - services → <Service>.Request
    • Auto-derive a full params dict from the class.
    • Overlay YAML-supplied param blocks so lock-ins win.
    • Return the hydrated entry, or None on import failure.
    """
    try:
        iface = import_ros_type(logger, entry["type"])
        if entry["interface"] == "action_client":
            iface = iface.Goal
        elif entry["interface"] == "service_client":
            iface = iface.Request
    except Exception as e:
        logger.error(f"[cap-scan] Drop capability '{entry['name']}' - cannot import {entry['type']}: {e}")
        return None

    auto_params  = extract_params_from_msg(logger, iface)
    locked       = entry.get("params", {})
    entry["params"] = merge_params(auto_params, locked)
    return entry


def build_manifest_from_caps(self, robot_caps: dict) -> dict:
    """
    Build a manifest that contains only the capabilities declared in
    robot_caps.yaml.  Non-builtin entries are hydrated (auto-derived
    params + YAML lock-ins).  Any entry whose interface class cannot
    be imported is dropped.
    """
    # create empty lists for every category defined in the constant
    manifest = {cat: [] for cat in CapabilityCategory.ordered()}

    for section in CapabilityCategory.ordered():
        # builtins are copied verbatim (no hydration)
        if section == CapabilityCategory.BUILTIN.value:
            manifest[section] = robot_caps.get(section, [])
            continue

        # topics / actions / services → hydrate + lock-in
        for cap in robot_caps.get(section, []):
            hydrated = _hydrate_entry(cap.copy(), self.get_logger())
            if hydrated:
                manifest[section].append(hydrated)

    return manifest


class CapabilityScannerNode(Node):

    def __init__(self):
        super().__init__('capability_scanner')
        # 1. Load YAML
        self.robot_caps = load_robot_capabilities(self)

        # 2. Build manifest purely from YAML (no live scan)
        final_manifest = build_manifest_from_caps(self, self.robot_caps)

        # 3. Publish once with TRANSIENT_LOCAL QoS
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

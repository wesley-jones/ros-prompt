import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
import json
import py_trees
import xml.etree.ElementTree as ET
import threading
from ros_prompt.adapters_py.generic_publisher_adapter import GenericPublisherAdapter
from ros_prompt.behaviors_py.generic_bt_behaviour import GenericPublisherBehaviour
from ros_prompt.utilities.ros_type_utils import import_ros_type
from ros_prompt.adapters_py.generic_action_adapter import GenericActionAdapter
from ros_prompt.behaviors_py.generic_action_behaviour import GenericActionBehaviour
import importlib

MODULE_PREFIX = "ros_prompt.adapters_py.builtins"

def load_class_from_manifest_entry(manifest_entry):
    file_name = manifest_entry['class_file']
    class_name = manifest_entry['plugin_name']
    module_name = f"{MODULE_PREFIX}.{file_name}"
    module = importlib.import_module(module_name)
    cls = getattr(module, class_name)
    return cls

def find_manifest_entry(tag, manifest_map):
    # Search topics
    for topic in manifest_map.get('topics', []):
        if topic.get('plugin_name') == tag or topic.get('name') == tag:
            return topic
    # Search actions
    for action in manifest_map.get('actions', []):
        if action.get('plugin_name') == tag or action.get('name') == tag:
            return action
    # Search builtins
    for builtin in manifest_map.get('builtins', []):
        if builtin.get('plugin_name') == tag:
            return builtin
    return None  # Not found

def str_to_type(type_str):
    """
    Convert a string type from the manifest to a Python type.
    Expand as needed for your types.
    """
    if type_str in [float, int, str, bool]:
        return type_str  # Already a Python type (if manifest is parsed with real types)
    mapping = {
        'float': float,
        'int': int,
        'str': str,
        'bool': lambda x: x in [True, "true", "True", 1, "1"],
    }
    return mapping[type_str]

def validate_and_coerce_attributes(tag, attrib, manifest_map):
    """
    Build the final {param_name: coerced_value} dict for this capability.

    Precedence (highest first):
        1. param_spec['value']        -> *hard constant* (overrides runtime)
        2. runtime XML attribute      -> user/operator override
        3. param_spec['default']      -> fallback
        (skip param if none of the above provides a value)

    If XML supplies a value that conflicts with a fixed constant
    (i.e. a 'value' entry in YAML), we keep the constant and log a warning.
    """
    cap = find_manifest_entry(tag, manifest_map)
    if not cap:
        raise ValueError(f"Capability/entry '{tag}' not found in manifest.")

    param_spec = cap.get('params', {})
    errors         = []
    coerced_params = {}

    for param_name, spec in param_spec.items():

        # --- 1. Figure out declared type ------------------------------------
        if isinstance(spec, dict):
            type_str = spec.get("type")
            if type_str is None:
                errors.append(f"Parameter '{param_name}' missing type in manifest")
                continue
        else:  # simple string like "float"
            type_str = spec

        # --- 2. Decide which raw value to use (precedence rules) ------------
        raw_value         = None
        runtime_provided  = param_name in attrib
        has_constant      = isinstance(spec, dict) and 'value' in spec
        has_default       = isinstance(spec, dict) and 'default' in spec

        if has_constant:
            raw_value = spec['value']
            if runtime_provided and attrib[param_name] != raw_value:
                # Warn but keep the constant
                print(  # or ros_node.get_logger().warn(...)
                    f"[{tag}] runtime attempted to override constant '{param_name}'. "
                    f"Ignoring runtime value '{attrib[param_name]}' and keeping '{raw_value}'."
                )
        elif runtime_provided:
            raw_value = attrib[param_name]
        elif has_default:
            raw_value = spec['default']
        else:
            # nothing to coerce for this param
            continue

        # --- 3. Coerce ------------------------------------------------------
        try:
            to_type        = str_to_type(type_str)
            coerced_value  = to_type(raw_value)
        except Exception as e:
            errors.append(
                f"Parameter '{param_name}' with value '{raw_value}' "
                f"could not be coerced to {type_str}: {e}"
            )
            continue

        coerced_params[param_name] = coerced_value

    # --- 4. Final validation -----------------------------------------------
    if errors:
        raise ValueError(
            f"Validation/coercion failed for capability '{tag}':\n" + "\n".join(errors)
        )

    return coerced_params



def create_behaviour_from_xml(tag, attrib, manifest_map, ros_node):
    try:
        params = validate_and_coerce_attributes(tag, attrib, manifest_map)
    except ValueError as e:
        ros_node.get_logger().error(str(e))
        return None

    cap = find_manifest_entry(tag, manifest_map)
    if not cap:
        ros_node.get_logger().error(f"Capability '{tag}' not found in manifest.")
        return None
    ros_node.get_logger().info(f"Found capability: {cap}")

    # Choose adapter/behaviour based on manifest info
    interface = cap.get("interface", "topic_publisher")  # default to publisher if missing

    if interface == "topic_publisher":
        msg_class = import_ros_type(ros_node, cap["type"])
        adapter = GenericPublisherAdapter(ros_node, cap["name"], msg_class)
        behaviour = GenericPublisherBehaviour(adapter, params, ros_node, name=tag)
    elif interface == "topic_subscriber":
        behaviour = None  # Placeholder for subscriber behaviour
        # msg_class = import_msg_class(ros_node, cap["type"])
        # adapter = GenericSubscriberAdapter(ros_node, cap["name"], msg_class)
        # behaviour = GenericSubscriberBehaviour(adapter, params, ros_node, name=tag)
    elif interface == "builtin":
        # Use plugin_name from manifest for lookup
        #try:
        behaviour_class = load_class_from_manifest_entry(cap)
        #except Exception as e:
         #   ros_node.get_logger().error(f"Failed to load builtin {cap['plugin_name']}: {e}")
          #  return None
        # Only pass params that the class expects (could add kwargs filtering here)
        behaviour = behaviour_class(**params, name=tag)
    elif interface == "action_client":
        action_class = import_ros_type(ros_node, cap["type"])  # For actions, this will be the action type (not FeedbackMessage)
        adapter = GenericActionAdapter(ros_node, cap["name"], action_class)
        behaviour = GenericActionBehaviour(adapter, params, ros_node, name=tag)
    else:
        ros_node.get_logger().error(f"Unknown capability interface: {interface}")
        return None

    return behaviour

class BTExecutorNode(Node):
    def __init__(self):
        super().__init__('bt_executor_node')
        self.capabilities = None
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.cap_sub = self.create_subscription(
            String,
            '/capabilities',
            self.capabilities_callback,
            qos_profile
        )

        self.current_tree = None
        self.current_tree_xml = None
        self.lock = threading.Lock()
        self.create_subscription(
            String,
            '/behavior_tree',
            self.bt_callback,
            10
        )
        # Start ticking BT in a separate thread
        self.bt_thread = threading.Thread(target=self.tick_loop)
        self.bt_thread.daemon = True
        self.bt_thread.start()

    def capabilities_callback(self, msg):
        try:
            self.capabilities = json.loads(msg.data)
            self.get_logger().info(f"Received capabilities: {self.capabilities}")
        except Exception as e:
            self.get_logger().error(f"Error parsing capabilities: {e}")

    def bt_callback(self, msg):
        self.get_logger().info(f"Received BT XML: {msg.data[:100]}...")  # Log first 100 chars for brevity
        # Parse and load the new BT if XML changed
        with self.lock:
            if msg.data != self.current_tree_xml:
                self.get_logger().info("Received new behavior tree XML.")
                new_tree = self.parse_bt_xml(msg.data)
                self.current_tree = new_tree
                self.current_tree_xml = msg.data
                self.get_logger().info(py_trees.display.unicode_tree(self.current_tree.root))

    def parse_bt_xml(self, xml_str):
        xml_str = self.clean_llm_xml(xml_str)
        root = ET.fromstring(xml_str)

        manifest_map = self.capabilities

        def build_tree(xml_node):
            tag = xml_node.tag
            attrib = xml_node.attrib

            # Handle composites (Sequence, Selector, etc.)
            if tag in ["Sequence", "Fallback", "Selector", "Parallel"]:
                # Map tag to the correct py_trees composite
                if tag == "Sequence":
                    behaviour = py_trees.composites.Sequence(name=attrib.get("name", "Sequence"), memory=False)
                elif tag in ["Selector", "Fallback"]:
                    behaviour = py_trees.composites.Selector(name=attrib.get("name", tag))
                elif tag == "Parallel":
                    behaviour = py_trees.composites.Parallel(name=attrib.get("name", "Parallel"))
                else:
                    raise ValueError(f"Unknown composite type: {tag}")

                for child_xml in xml_node:
                    child_behaviour = build_tree(child_xml)
                    behaviour.add_child(child_behaviour)
                return behaviour
            else:
                # Leaf node: create adapter/builtin from manifest
                # Pass 'self' as the ros_node reference
                new_behavior = create_behaviour_from_xml(tag, attrib, manifest_map, self)
                if new_behavior is None:
                    raise ValueError(f"Failed to create behaviour for tag '{tag}' with attributes {attrib}")
                return py_trees.decorators.OneShot(
                    name=f"OneShot_{new_behavior.name}",
                    child=new_behavior,
                    policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
                )

        # If the root is <BehaviorTree>, skip to its first child
        if root.tag == "BehaviorTree":
            # Typically, BT XML wraps everything in a <BehaviorTree> root
            if len(root) == 0:
                raise ValueError("No child node found under <BehaviorTree>")
            # The actual tree root is the first child (e.g., <Sequence>)
            tree_root = build_tree(root[0])
        else:
            tree_root = build_tree(root)

        return py_trees.trees.BehaviourTree(tree_root)

    def tick_loop(self):
        rate = self.create_rate(0.2)  # 0.2 Hz
        while rclpy.ok():
            self.get_logger().info("Tick loop running...")
            with self.lock:
                if self.current_tree:
                    for node in self.current_tree.root.iterate():
                        self.get_logger().info(f"## Before ### Node: {node.name}, Status: {node.status}")
                    self.current_tree.tick()
                    status = self.current_tree.root.status
                    self.get_logger().info(f"Tick returned status: {status}")
                    for node in self.current_tree.root.iterate():
                        self.get_logger().info(f"## After ### Node: {node.name}, Status: {node.status}")
                    if status in [py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE]:
                        self.get_logger().info("Tree complete; clearing current_tree.")
                        self.current_tree = None
                        self.current_tree_xml = None

            rate.sleep()
    
    import re

    def clean_llm_xml(self, raw_llm_output):
        import re
        # Step 1: Remove markdown fencing
        xml = re.sub(r'^```xml\n?', '', raw_llm_output)   # Remove opening fence
        xml = re.sub(r'\n?```$', '', xml)                # Remove closing fence

        # Step 2: Unescape if it's JSON-encoded
        xml = xml.encode('utf-8').decode('unicode_escape')

        # Step 3: Strip any leading/trailing whitespace
        xml = xml.strip()
        return xml


def main(args=None):
    rclpy.init(args=args)
    node = BTExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

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
from ros_prompt.utilities.bt_builder import build_py_tree
from ros_prompt.adapters_py.generic_action_adapter import GenericActionAdapter
from ros_prompt.behaviors_py.generic_action_behaviour import GenericActionBehaviour
from ros_prompt.utilities.manifest_helpers import find_manifest_entry, validate_and_coerce_attributes, load_class_from_manifest_entry

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
        self.bt_schema = None
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.cap_sub = self.create_subscription(
            String,
            '/capabilities',
            self.capabilities_callback,
            qos_profile
        )

        self.current_tree = None
        self.current_tree_json = None
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
        self.get_logger().info(f"Received BT json: {msg.data[:100]}...")  # Log first 100 chars for brevity
        # Parse and load the new BT if JSON changed
        with self.lock:
            if msg.data != self.current_tree_json:
                self.get_logger().info("Received new behavior tree json.")
                my_dict = json.loads(msg.data)
                new_tree = self.parse_bt_json(my_dict)
                # new_tree = self.parse_bt_json(msg.data)
                self.current_tree = new_tree
                self.current_tree_json = msg.data
                self.get_logger().info(py_trees.display.unicode_tree(self.current_tree.root))

    def parse_bt_json(self, json_str):

        bt_root = llm_bt_to_pytrees(json_str)

        # try:
        #     bt_pydantic_obj = self.bt_schema.model_validate_json(json_str)
        # except Exception as e:
        #     self.get_logger().error(f"Failed to validate BT JSON: {e}")
        #     return
        # bt_root = build_py_tree(
        #     node = bt_pydantic_obj,
        #     manifest_map=self.capabilities,
        #     ros_node=self
        # )
        self.tree = py_trees.trees.BehaviourTree(bt_root)

        # Convert JSON to py_trees structure
        return self.tree

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

def llm_bt_to_pytrees(node):
    node_type = node["type"]
    name = node.get("name", node_type)
    if node_type.lower() == "sequence":
        tree_node = py_trees.composites.Sequence(name=name)
    elif node_type.lower() == "selector":
        tree_node = py_trees.composites.Selector(name=name)
    elif node_type.lower() == "action":
        tree_node = py_trees.behaviours.Success(name=name)  # Replace with real robot actions!
    else:
        raise ValueError(f"Unknown node type: {node_type}")
    for child in node.get("children", []):
        tree_node.add_child(llm_bt_to_pytrees(child))
    return tree_node

def main(args=None):
    rclpy.init(args=args)
    node = BTExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

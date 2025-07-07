import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
import json
import py_trees
import xml.etree.ElementTree as ET
import threading
from ros_prompt.utilities.generic_publisher_adapter import GenericPublisherAdapter
from ros_prompt.utilities.generic_bt_behaviour import GenericPublisherBehaviour
from ros_prompt.utilities.ros_type_utils import import_msg_class

def find_manifest_entry(tag, manifest_map):
    # Search topics
    for topic in manifest_map.get('topics', []):
        if topic.get('plugin_name') == tag or topic.get('name') == tag:
            return topic
    # Search builtins
    for builtin in manifest_map.get('builtins', []):
        if builtin.get('name') == tag:
            return builtin
    return None  # Not found


def create_behaviour_from_xml(tag, attrib, manifest_map, ros_node):
    cap = find_manifest_entry(tag, manifest_map)
    if not cap:
        ros_node.get_logger().error(f"Capability '{tag}' not found in manifest.")
        return None
    ros_node.get_logger().info(f"Found capability: {cap}")
    ros_node.get_logger().info(f"Creating behaviour for capability: {cap['name']} ({cap['type']})")
    msg_class = import_msg_class(ros_node, cap["type"])
    adapter = GenericPublisherAdapter(ros_node, cap["name"], msg_class)
    behaviour = GenericPublisherBehaviour(adapter, attrib, name=tag)
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
        # Parse and load the new BT if XML changed
        with self.lock:
            if msg.data != self.current_tree_xml:
                # try:
                    self.get_logger().info("Received new behavior tree XML.")
                    new_tree = self.parse_bt_xml(msg.data)
                    self.current_tree = new_tree
                    self.current_tree_xml = msg.data
                    self.get_logger().info(py_trees.display.unicode_tree(self.current_tree))

                # except Exception as e:
                    # self.get_logger().error(f"Failed to parse BT: {e}")

    def parse_bt_xml(self, xml_str):
        temp_xml_str = """<BehaviorTree ID="MainTree">
  <Sequence>
    <SetCmd_vel linear_x="0.2" linear_y="0.0" linear_z="0.0" angular_x="0.0" angular_y="0.0" angular_z="0.0"/>
    <SetCmd_vel linear_x="0.0" linear_y="0.0" linear_z="0.0" angular_x="0.0" angular_y="0.0" angular_z="0.0"/>
  </Sequence>
</BehaviorTree>
"""
        temp_xml_str_with_wait = """<BehaviorTree ID="MainTree">
  <Sequence>
    <SetCmd_vel linear_x="0.2" linear_y="0.0" linear_z="0.0" angular_x="0.0" angular_y="0.0" angular_z="0.0"/>
    <Wait duration="5"/>
    <SetCmd_vel linear_x="0.0" linear_y="0.0" linear_z="0.0" angular_x="0.0" angular_y="0.0" angular_z="0.0"/>
  </Sequence>
</BehaviorTree>
"""
        xml_str = self.clean_llm_xml(temp_xml_str)
        # Minimal example: parses <sequence> with <action> children
        self.get_logger().info("Behavior Tree XML:\n" + xml_str)
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
                return create_behaviour_from_xml(tag, attrib, manifest_map, self)

        # If the root is <BehaviorTree>, skip to its first child
        if root.tag == "BehaviorTree":
            # Typically, BT XML wraps everything in a <BehaviorTree> root
            if len(root) == 0:
                raise ValueError("No child node found under <BehaviorTree>")
            # The actual tree root is the first child (e.g., <Sequence>)
            tree_root = build_tree(root[0])
        else:
            tree_root = build_tree(root)

        return tree_root

    def tick_loop(self):
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok():
            with self.lock:
                if self.current_tree:
                    self.current_tree.tick()
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


# Example action for testing
class SayAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        print(f"[ACTION] Executing {self.name}")
        return py_trees.common.Status.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    node = BTExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import py_trees
import xml.etree.ElementTree as ET
import threading

class BTExecutorNode(Node):
    def __init__(self):
        super().__init__('bt_executor_node')
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

    def bt_callback(self, msg):
        # Parse and load the new BT if XML changed
        with self.lock:
            if msg.data != self.current_tree_xml:
                try:
                    self.get_logger().info("Received new behavior tree XML.")
                    new_tree = self.parse_bt_xml(msg.data)
                    self.current_tree = new_tree
                    self.current_tree_xml = msg.data
                except Exception as e:
                    self.get_logger().error(f"Failed to parse BT: {e}")

    def parse_bt_xml(self, xml_str):
        xml_str = self.clean_llm_xml(xml_str)
        # Minimal example: parses <sequence> with <action> children
        self.get_logger().info("Behavior Tree XML:\n" + xml_str)
        root = ET.fromstring(xml_str)
        main_seq = None

        # Find main sequence (your XML may differ)
        for child in root.iter():
            if child.tag.lower() == "sequence":
                main_seq = child
                break
        if main_seq is None:
            raise ValueError("No <sequence> found in BT XML!")

        # Build py_trees BT
        seq = py_trees.composites.Sequence(name="MainSequence", memory=False)
        for act in main_seq:
            if act.tag == "action":
                seq.add_child(SayAction(act.attrib.get("name", "UnnamedAction")))
            # Add more node types as needed (e.g., Condition, Service)
        tree = py_trees.trees.BehaviourTree(seq)
        return tree

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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
import json
import py_trees
import xml.etree.ElementTree as ET
import threading
from ros_prompt.utilities.bt_builder import parse_bt_dict

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
        self.get_logger().info(f"Received BT json: {msg.data}")
        # Parse and load the new BT if JSON changed
        with self.lock:
            if msg.data != self.current_tree_json:
                bt_dict = json.loads(msg.data)
                new_tree = parse_bt_dict(self=self, bt_dict=bt_dict, manifest_map=self.capabilities)
                self.current_tree = new_tree
                self.current_tree_json = msg.data
                self.get_logger().info(py_trees.display.unicode_tree(self.current_tree.root))

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

def main(args=None):
    rclpy.init(args=args)
    node = BTExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

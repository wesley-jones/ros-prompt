import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from rclpy.qos import QoSProfile, DurabilityPolicy
from ros_prompt.utilities.llm_client import LLMClient

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.capabilities = None
        self.world_state = self.get_initial_world_state()  # You might want to subscribe for updates later
        self.llm_client = LLMClient(node=self)

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.cap_sub = self.create_subscription(
            String,
            '/capabilities',
            self.capabilities_callback,
            qos_profile
        )
        self.intent_sub = self.create_subscription(
            String,
            '/intent',
            self.intent_callback,
            10
        )
        self.bt_pub = self.create_publisher(
            String,
            '/behavior_tree',
            10
        )

    def capabilities_callback(self, msg):
        try:
            self.capabilities = json.loads(msg.data)
            self.get_logger().info(f"Received capabilities: {self.capabilities}")
        except Exception as e:
            self.get_logger().error(f"Error parsing capabilities: {e}")

    def get_initial_world_state(self):
        # For now, just stub in a fixed state
        return {
            "battery": "full",
            "location": "charging_dock",
            "capabilities": ["navigate", "pickup", "speak"]
        }

    def intent_callback(self, msg):
        if self.capabilities is None:
            self.get_logger().warn("Capabilities not yet received. Skipping planning.")
            return
        try:
            self.get_logger().info("Processing intent...")
            intent = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Intent processing error: {e}")
            return

        self.get_logger().info(f"Received intent: {intent}")

        try:
            bt_xml = self.llm_client.query_llm(
                system_prompt=self.get_system_prompt(),
                user_prompt=f"Intent: {intent}",
                final_instructions="Respond ONLY with the XML, no extra explanation.",
                world_state=self.world_state,
                max_tokens=1024,
                temperature=0.2,
                timeout=300
            )
            self.get_logger().info(f"Generated BT XML: {bt_xml}")
        except Exception as e:
            self.get_logger().error(f"Error querying LLM: {e}")

        try:
            # Assuming bt_xml is a string containing the XML
            if bt_xml:
                self.publish_behavior_tree(bt_xml)
        except Exception as e:
            self.get_logger().error(f"Error generating behavior tree: {e}")

    def get_system_prompt(self):
        system_prompt = (
            "Given the following robot intent, capabilities, and world state, "
            "generate a behavior tree in BT.CPP/Nav2 XML format that the robot can execute."
            "Respond ONLY with the XML, no extra explanation."
        )
        return system_prompt

    def publish_behavior_tree(self, bt_xml):
        msg = String()
        msg.data = bt_xml
        self.bt_pub.publish(msg)
        self.get_logger().info("Published new behavior tree.")

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

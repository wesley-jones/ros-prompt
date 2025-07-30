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
        # self.world_state = self.get_initial_world_state()  # You might want to subscribe for updates later
        self.llm_client = LLMClient(node=self)

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.cap_sub = self.create_subscription(
            String,
            '/capabilities',
            self.capabilities_callback,
            qos_profile
        )
        self.subscription = self.create_subscription(
            String,
            '/user_input',
            self.user_input_callback,
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

    def user_input_callback(self, msg):
        user_text = msg.data
        self.get_logger().info(f"Received user input: {user_text}")
        
        if self.capabilities is None:
            self.get_logger().warn("Capabilities not yet received. Skipping planning.")
            return

        behavior_tree_dict = self.llm_client.query_langchain(user_text, self.capabilities)

        self.get_logger().info(f"Generated BT: {behavior_tree_dict}")

        if behavior_tree_dict:
            bt_json = json.dumps(behavior_tree_dict, indent=2)
            self.publish_behavior_tree(bt_json)
       
    def get_system_prompt(self):
        system_prompt = (
            "Given the following user request and capabilities, "
            "generate a behavior tree in BT.CPP/Nav2 XML format that the robot can execute."
            "Respond ONLY with the XML, no extra explanation."
            " Here are the robot's capabilities:\n"
            f"{json.dumps(self.capabilities, indent=2)}\n\n"
        )
        return system_prompt

    def publish_behavior_tree(self, bt_json):
        msg = String()
        msg.data = bt_json
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

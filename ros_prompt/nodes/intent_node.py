import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from ros_prompt.utilities.llm_client import LLMClient

class IntentNode(Node):
    def __init__(self):
        super().__init__('intent_node')
        self.get_logger().info("Initializing IntentNode...")
        # Initialize the LLM client
        self.llm_client = LLMClient(node=self)
        # self.get_logger().info("LLMClient initialized.")
        self.subscription = self.create_subscription(
            String,
            '/user_input',
            self.user_input_callback,
            10)
        self.publisher = self.create_publisher(
            String,
            '/intent',
            10)
        self.get_logger().info("IntentNode initialized and ready to process user input.")

    def get_world_state(self):
        # For now, just return a simple dict. Replace with actual state fetch.
        # E.g., read from file, param, or a service call.
        return {
            "battery": "full",
            "location": "charging_dock",
            "capabilities": ["navigate", "pickup", "speak"]
        }

    def user_input_callback(self, msg):
        user_text = msg.data
        self.get_logger().info(f"Received user input: {user_text}")

        intent = self.llm_client.query_llm(
            system_prompt=self.get_system_prompt(),
            final_instructions=self.get_final_instructions(),
            user_prompt=f"User: {user_text}",
            world_state=self.get_world_state()
        )
        self.get_logger().info(f"LLM response: {intent}")
        if intent:
            self.publish_intent(intent)

    def get_system_prompt(self):
        # You can make this much more sophisticated!
        system_prompt = ("Given the following user request and current robot world state, "
            "extract the intent as a JSON object.\n")
        return system_prompt

    def get_final_instructions(self):
        final_instructions = ("Respond only with a JSON intent, e.g., "
            '{"intent": "navigate", "target": "kitchen"}')
        return final_instructions

    def publish_intent(self, intent_text):
        msg = String()
        msg.data = intent_text
        self.publisher.publish(msg)
        self.get_logger().info(f"Published intent: {intent_text}")

def main(args=None):
    rclpy.init(args=args)
    node = IntentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

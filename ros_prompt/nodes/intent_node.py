import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json

LLAMA_SERVER_URL = "http://localhost:8000/v1/completions"  # Update if different
LLM_MODEL_NAME = "mistral"  # Change to your model's id if needed

class IntentNode(Node):
    def __init__(self):
        super().__init__('intent_node')
        self.declare_parameter('llm_url', LLAMA_SERVER_URL)
        self.llm_url = self.get_parameter('llm_url').get_parameter_value().string_value
        self.world_state = self.load_world_state()
        
        self.subscription = self.create_subscription(
            String,
            '/user_input',
            self.user_input_callback,
            10)
        self.publisher = self.create_publisher(
            String,
            '/intent',
            10)

    def load_world_state(self):
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

        prompt = self.build_prompt(user_text)
        intent = self.query_llm(prompt)
        if intent:
            self.publish_intent(intent)

    def build_prompt(self, user_text):
        # You can make this much more sophisticated!
        prompt = (
            "Given the following user request and current robot world state, "
            "extract the intent as a JSON object.\n"
            f"User: {user_text}\n"
            f"World state: {json.dumps(self.world_state)}\n"
            "Respond only with a JSON intent, e.g., "
            '{"intent": "navigate", "target": "kitchen"}'
        )
        return prompt

    def query_llm(self, prompt):
        headers = {"Content-Type": "application/json"}
        payload = {
            "model": LLM_MODEL_NAME,
            "prompt": prompt,
            "max_tokens": 256,
            "temperature": 0.2
        }
        try:
            response = requests.post(self.llm_url, headers=headers, json=payload, timeout=20)
            response.raise_for_status()
            # Response from llama_cpp.server might vary, check the docs for the actual output format.
            data = response.json()
            # Usually, the completion text is under 'choices'
            completion = data['choices'][0]['text']
            self.get_logger().info(f"LLM intent: {completion}")
            return completion.strip()
        except Exception as e:
            self.get_logger().error(f"LLM request failed: {e}")
            return None

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

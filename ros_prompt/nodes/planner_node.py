import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import yaml
import json
import os

LLAMA_SERVER_URL = "http://localhost:8000/v1/completions"
LLM_MODEL_NAME = "mistral"
CAPS_FILE = os.path.expanduser("~/ros2_ws/src/ros_prompt/config/robot_caps.yaml")  # Update path as needed

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.capabilities = self.load_capabilities(CAPS_FILE)
        self.world_state = self.get_initial_world_state()  # You might want to subscribe for updates later

        self.subscription = self.create_subscription(
            String,
            '/intent',
            self.intent_callback,
            10
        )
        self.bt_publisher = self.create_publisher(
            String,
            '/behavior_tree',
            10
        )

    def load_capabilities(self, path):
        try:
            with open(path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Could not load capabilities: {e}")
            return {}

    def get_initial_world_state(self):
        # For now, just stub in a fixed state
        return {
            "battery": "full",
            "location": "charging_dock"
        }

    def intent_callback(self, msg):
        try:
            intent = json.loads(msg.data)
            self.get_logger().info(f"Received intent: {intent}")

            prompt = self.build_prompt(intent)
            bt_xml = self.query_llm(prompt)

            if bt_xml:
                self.publish_behavior_tree(bt_xml)
        except Exception as e:
            self.get_logger().error(f"Intent processing error: {e}")

    def build_prompt(self, intent):
        # You may want to tune this for best LLM performance
        prompt = (
            "Given the following robot intent, capabilities, and world state, "
            "generate a behavior tree in BT.CPP/Nav2 XML format that the robot can execute.\n"
            f"Intent: {json.dumps(intent)}\n"
            f"Capabilities: {json.dumps(self.capabilities)}\n"
            f"World state: {json.dumps(self.world_state)}\n"
            "Respond ONLY with the XML, no extra explanation."
        )
        return prompt

    def query_llm(self, prompt):
        headers = {"Content-Type": "application/json"}
        payload = {
            "model": LLM_MODEL_NAME,
            "prompt": prompt,
            "max_tokens": 1024,
            "temperature": 0.2
        }
        try:
            response = requests.post(LLAMA_SERVER_URL, headers=headers, json=payload, timeout=30)
            response.raise_for_status()
            data = response.json()
            completion = data['choices'][0]['text']
            self.get_logger().info("Received behavior tree from LLM.")
            return completion.strip()
        except Exception as e:
            self.get_logger().error(f"LLM request failed: {e}")
            return None

    def publish_behavior_tree(self, bt_xml):
        msg = String()
        msg.data = bt_xml
        self.bt_publisher.publish(msg)
        self.get_logger().info("Published new behavior tree.")

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

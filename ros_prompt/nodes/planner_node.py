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

        # try:
        pydantic_bt = self.llm_client.query_langchain(user_text, self.capabilities)
#             bt_xml = self.llm_client.query_llm(
#                 system_prompt=self.get_system_prompt(),
#                 user_prompt=f"User Request: {user_text}",
#                 final_instructions="""You must use the plugin_name value from the manifest as the XML tag name for each behavior/action.
# Do NOT use generic tags such as <Action>. Use only the plugin_name from the capabilities manifest for each node. Here is an example:

# <BehaviorTree ID="MainTree">
#   <Sequence>
#     <SetCmd_vel linear_x="0.2" linear_y="0.0" linear_z="0.0" angular_x="0.0" angular_y="0.0" angular_z="0.0"/>
#     <CommandHold duration="5"/>
#     <SetCmd_vel linear_x="0.0" linear_y="0.0" linear_z="0.0" angular_x="0.0" angular_y="0.0" angular_z="0.0"/>
#   </Sequence>
# </BehaviorTree>

# Here is another example:

# <BehaviorTree ID="MainTree">
#   <NavigateToPose
#     pose_pose_position_x="0.0"
#     pose_pose_position_y="0.0"
#     pose_pose_position_z="0.0"
#     pose_pose_orientation_x="0.0"
#     pose_pose_orientation_y="0.0"
#     pose_pose_orientation_z="0.0"
#     pose_pose_orientation_w="1.0"/>
# </BehaviorTree>

# Respond ONLY with the XML, no extra explanation.""",
#                 # world_state=self.world_state,
#                 max_tokens=1024,
#                 temperature=0.2,
#                 timeout=300
#             )
        self.get_logger().info(f"Generated BT: {pydantic_bt}")
        # except Exception as e:
        #     self.get_logger().error(f"Error querying LLM: {e}")
        #     return

        # try:
            # Assuming bt is a pydantic model
        if pydantic_bt:
            # bt_json = pydantic_bt.model_dump_json()
            bt_json = json.dumps(pydantic_bt, indent=2)
            self.publish_behavior_tree(bt_json)
        # except Exception as e:
        #     self.get_logger().error(f"Error generating behavior tree: {e}")

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

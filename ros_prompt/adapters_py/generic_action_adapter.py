from rclpy.action import ActionClient
from ros_prompt.utilities.ros_type_utils import set_nested_attr

class GenericActionAdapter:
    def __init__(self, node, action_name, action_type):
        self.node = node
        self.action_name = action_name
        self.action_type = action_type
        self.client = ActionClient(node, action_type, action_name)
        self.goal_handle = None
        self.result_future = None
        self.status = "idle"
        self.result = None

    def execute(self, **goal_kwargs):
        # Build goal message
        goal_msg = self.action_type.Goal()
        # Fill fields (can improve this for nested fields if needed)
        for k, v in goal_kwargs.items():
            set_nested_attr(goal_msg, k, v)
        self.status = "pending"
        self.client.wait_for_server()
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.status = "rejected"
            self.node.get_logger().error("Goal was rejected by the action server.")
            return
        self.goal_handle = goal_handle
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self._result_callback)
        self.status = "active"

    def _result_callback(self, future):
        result = future.result().result
        self.result = result
        self.status = "succeeded"
        self.node.get_logger().info(f"Action result: {result}")

    def check_status(self):
        return self.status

    def get_result(self):
        return self.result

    def cancel_goal(self):
        if self.goal_handle and self.status in ["active", "pending"]:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(lambda f: self.node.get_logger().info("Goal cancelled"))
        self.status = "cancelled"

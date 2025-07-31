from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
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

    def execute(self, timeout=30.0, **goal_kwargs):
        self.node.get_logger().info(f"Sending goal to action '{self.action_name}' with params: {goal_kwargs}")
        # Build goal message
        goal_msg = self.action_type.Goal()
        # Fill fields (can improve this for nested fields if needed)
        for k, v in goal_kwargs.items():
            set_nested_attr(goal_msg, k, v)
        self.status = "pending"
        success = self.client.wait_for_server(timeout_sec=timeout)
        if not success:
            self.node.get_logger().error(f"Timed out waiting for action server '{self.action_name}'")
            self.status = "rejected"
            return
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        self.node.get_logger().info(f"Received goal handle: {goal_handle}")
        if not goal_handle.accepted:
            self.status = "rejected"
            self.node.get_logger().error("Goal was rejected by the action server.")
            return
        self.goal_handle = goal_handle
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self._result_callback)
        self.node.get_logger().info(f"Setting '{self.action_name}' status to 'active'")
        self.status = "active"

    def _result_callback(self, future):
        result_obj = future.result()
        status = result_obj.status
        result_msg = result_obj.result
        self.node.get_logger().info(
            f"Action '{self.action_name}' completed. Status: {status}, Result: {result_msg}"
        )
        self.result = result_msg
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.status = "succeeded"
        elif status == GoalStatus.STATUS_ABORTED:
            self.status = "aborted"
        elif status == GoalStatus.STATUS_CANCELED:
            self.status = "cancelled"
        else:
            # This covers unknown, accepted, executing, canceling, etc.
            self.status = "unknown"
            self.node.get_logger().warn(f"Unexpected action status {status} for '{self.action_name}'")
        self.node.get_logger().info(f"Action '{self.action_name}' status updated to: {self.status}")

    def check_status(self):
        self.node.get_logger().info(f"Checking status of action '{self.action_name}': {self.status}")
        return self.status

    def cancel_goal(self):
        if self.goal_handle and self.status in ["active", "pending"]:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(lambda f: self.node.get_logger().info("Goal cancelled"))
        self.status = "cancelled"

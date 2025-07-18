from ros_prompt.utilities.ros_type_utils import set_nested_attr

class GenericPublisherAdapter():
    def __init__(self, node, topic_name, msg_type):
        self.node = node
        self.node.get_logger().info(f"Creating GenericPublisherAdapter for topic: {topic_name} with type: {msg_type}")
        self.publisher = node.create_publisher(msg_type, topic_name, 10)

    def execute(self, **kwargs):
        self.node.get_logger().info(f"Publishing message to {self.publisher.topic_name} with data: {kwargs}")
        msg = self._create_msg(**kwargs)
        self.publisher.publish(msg)

    def _create_msg(self, **kwargs):
        # This should construct a msg of the right type and fill in fields from kwargs
        # For geometry_msgs/Twist, map kwargs to .linear.x, .angular.z, etc.
        msg = self.publisher.msg_type()  # or use a helper
        for field, value in kwargs.items():
            set_nested_attr(msg, field, value)
        return msg

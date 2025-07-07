class GenericPublisherAdapter:
    def __init__(self, node, topic_name, msg_type):
        self.publisher = node.create_publisher(msg_type, topic_name, 10)

    def execute(self, **kwargs):
        msg = self._create_msg(**kwargs)
        self.publisher.publish(msg)

    def _create_msg(self, **kwargs):
        # This should construct a msg of the right type and fill in fields from kwargs
        # For geometry_msgs/Twist, map kwargs to .linear.x, .angular.z, etc.
        msg = self.publisher.msg_type()  # or use a helper
        for field, value in kwargs.items():
            # Handle nested fields, e.g., 'linear_x' -> msg.linear.x
            parts = field.split('_')
            current = msg
            for part in parts[:-1]:
                current = getattr(current, part)
            setattr(current, parts[-1], value)
        return msg

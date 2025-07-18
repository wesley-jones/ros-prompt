import importlib

def import_ros_type(self, msg_type_str):
    # Accepts "geometry_msgs/msg/Vector3" or "geometry_msgs/Vector3" or "nav2_msgs/action/NavigateToPose"
    parts = msg_type_str.split('/')
    if len(parts) == 2:
        # e.g., 'geometry_msgs/Vector3' → 'geometry_msgs/msg/Vector3'
        pkg, msg = parts
        msg_type_str = f"{pkg}/msg/{msg}"
        parts = msg_type_str.split('/')
    if len(parts) == 3:
        pkg, subfolder, class_name = parts  # subfolder: 'msg' or 'action'
    else:
        raise ValueError(f"Unexpected ROS message type string: {msg_type_str}")
    module_name = f"{pkg}.{subfolder}"
    mod = importlib.import_module(module_name)
    return getattr(mod, class_name)

def set_nested_attr(obj, attr_path, value):
    """
    Sets a (possibly nested) attribute of a ROS message object.
    E.g., set_nested_attr(msg, "pose_pose_position_x", 1.0) sets msg.pose.pose.position.x = 1.0
    """
    parts = attr_path.split('_')
    current = obj
    for part in parts[:-1]:
        current = getattr(current, part)
    setattr(current, parts[-1], value)

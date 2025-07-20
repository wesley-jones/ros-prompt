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

def set_nested_attr(msg, attr_path, value):
    """
    Works for flattened paths like
        pose_header_frame_id
    even when the last attribute itself contains an underscore
    (frame_id, stamp_sec, orientation_w, etc.).

    Greedy algorithm, but stops at the *parent* of the final field so
    we can call setattr once.
    """
    parts   = attr_path.split('_')
    current = msg

    while parts:
        # find the longest prefix that is an attribute of 'current'
        for i in range(len(parts), 0, -1):
            candidate = '_'.join(parts[:i])
            if hasattr(current, candidate):
                # Is this the *final* attribute?
                if len(parts) == i:
                    setattr(current, candidate, value)
                    return
                # Otherwise, descend and continue
                current = getattr(current, candidate)
                parts   = parts[i:]
                break
        else:
            raise AttributeError(
                f"Cannot resolve '{attr_path}': "
                f"no attribute '{'_'.join(parts)}' on {type(current).__name__}"
            )

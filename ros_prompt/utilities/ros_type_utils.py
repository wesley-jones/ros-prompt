import importlib

def import_ros_type(self, type_str: str):
    """
    Accepts either fully-qualified   "pkg/{msg|srv|action}/Type"
    or the two-token shorthand       "pkg/Type" (assumed msg).

    Returns the generated Python class for that interface.
    """
    parts = type_str.split('/')

    # ── Shorthand: default to messages ─────────────────────────────
    if len(parts) == 2:                       # e.g. "geometry_msgs/Vector3"
        pkg, cls = parts
        subfolder = "msg"
    # ── Fully-qualified form ───────────────────────────────────────
    elif len(parts) == 3:                     # e.g. "nav2_msgs/srv/SaveMap"
        pkg, subfolder, cls = parts
    else:
        raise ValueError(f"Unexpected ROS type string: {type_str}")

    if subfolder not in ("msg", "srv", "action"):
        raise ValueError(f"Unsupported ROS interface: {subfolder}")

    module = importlib.import_module(f"{pkg}.{subfolder}")
    return getattr(module, cls)

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

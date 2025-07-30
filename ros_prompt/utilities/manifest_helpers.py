import importlib

MODULE_PREFIX = "ros_prompt.adapters_py.builtins"

def find_manifest_entry(tag, manifest_map):
    # Search topics
    for topic in manifest_map.get('topics', []):
        if topic.get('name') == tag or topic.get('name') == tag:
            return topic
    # Search actions
    for action in manifest_map.get('actions', []):
        if action.get('name') == tag or action.get('name') == tag:
            return action
    # Search builtins
    for builtin in manifest_map.get('builtins', []):
        if builtin.get('name') == tag:
            return builtin
    return None  # Not found

def validate_and_coerce_attributes(tag, attrib, manifest_map):
    """
    Build the final {param_name: coerced_value} dict for this capability.

    Precedence (highest first):
        1. param_spec['value']        -> *hard constant* (overrides runtime)
        2. runtime XML attribute      -> user/operator override
        3. param_spec['default']      -> fallback
        (skip param if none of the above provides a value)

    If XML supplies a value that conflicts with a fixed constant
    (i.e. a 'value' entry in YAML), we keep the constant and log a warning.
    """
    cap = find_manifest_entry(tag, manifest_map)
    if not cap:
        raise ValueError(f"Capability/entry '{tag}' not found in manifest.")

    param_spec = cap.get('params', {})
    errors         = []
    coerced_params = {}

    for param_name, spec in param_spec.items():

        # --- 1. Figure out declared type ------------------------------------
        if isinstance(spec, dict):
            type_str = spec.get("type")
            if type_str is None:
                errors.append(f"Parameter '{param_name}' missing type in manifest")
                continue
        else:  # simple string like "float"
            type_str = spec

        # --- 2. Decide which raw value to use (precedence rules) ------------
        raw_value         = None
        runtime_provided  = param_name in attrib
        has_constant      = isinstance(spec, dict) and 'value' in spec
        has_default       = isinstance(spec, dict) and 'default' in spec

        if has_constant:
            raw_value = spec['value']
            if runtime_provided and attrib[param_name] != raw_value:
                # Warn but keep the constant
                print(  # or ros_node.get_logger().warn(...)
                    f"[{tag}] runtime attempted to override constant '{param_name}'. "
                    f"Ignoring runtime value '{attrib[param_name]}' and keeping '{raw_value}'."
                )
        elif runtime_provided:
            raw_value = attrib[param_name]
        elif has_default:
            raw_value = spec['default']
        else:
            # nothing to coerce for this param
            continue

        # --- 3. Coerce ------------------------------------------------------
        try:
            to_type        = str_to_type(type_str)
            coerced_value  = to_type(raw_value)
        except Exception as e:
            errors.append(
                f"Parameter '{param_name}' with value '{raw_value}' "
                f"could not be coerced to {type_str}: {e}"
            )
            continue

        coerced_params[param_name] = coerced_value

    # --- 4. Final validation -----------------------------------------------
    if errors:
        raise ValueError(
            f"Validation/coercion failed for capability '{tag}':\n" + "\n".join(errors)
        )

    return coerced_params

def load_class_from_manifest_entry(manifest_entry):
    file_name = manifest_entry['class_file']
    class_name = manifest_entry['name']
    module_name = f"{MODULE_PREFIX}.{file_name}"
    module = importlib.import_module(module_name)
    cls = getattr(module, class_name)
    return cls

def import_ros_type(msg_type_str):
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
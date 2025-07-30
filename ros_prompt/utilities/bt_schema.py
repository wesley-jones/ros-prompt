# schema_builder.py

def json_type(manifest_type):
    """Map manifest type to JSON Schema type."""
    if manifest_type == 'float':
        return 'number'
    elif manifest_type == 'str':
        return 'string'
    elif manifest_type == 'int':
        return 'integer'
    elif manifest_type == 'bool':
        return 'boolean'
    # Expand as needed
    return 'string'  # fallback

def make_action_schema(action, is_topic=False):
    name = action["name"]
    # Prepare parameters object
    parameters = action["params"]
    props = {}
    required = []
    for p_name, p_spec in parameters.items():
        if isinstance(p_spec, dict):
            p_type = p_spec.get("type", "string")
        else:
            p_type = p_spec  # if manifest has bare type string
        props[p_name] = {"type": json_type(p_type)}
        required.append(p_name)
    return {
        "type": "object",
        "description": action.get("description", ""),
        "properties": {
            "type": {"type": "string", "enum": ["Action"]},
            "name": {"type": "string", "enum": [name]},
            "parameters": {
                "type": "object",
                "properties": props,
                "required": required,
                "additionalProperties": False
            }
        },
        "required": ["type", "name", "parameters"],
        "additionalProperties": False
    }

def build_behavior_tree_schema(manifest):
    # Static Sequence/Selector nodes
    composites = [
        {
            "type": "object",
            "description": "A Sequence node: runs child nodes in order, succeeding only if all children succeed.",
            "properties": {
                "type": { "type": "string", "enum": ["Sequence"] },
                "children": {
                    "type": "array",
                    "items": { "$ref": "#/$defs/BTNode" }
                }
            },
            "required": ["type", "children"],
            "additionalProperties": False
        },
        {
            "type": "object",
            "description": "A Selector node: runs child nodes in order, succeeding if any child succeeds.",
            "properties": {
                "type": { "type": "string", "enum": ["Selector"] },
                "children": {
                    "type": "array",
                    "items": { "$ref": "#/$defs/BTNode" }
                }
            },
            "required": ["type", "children"],
            "additionalProperties": False
        },
    ]
    # Action nodes from topics
    topic_actions = [make_action_schema(topic, is_topic=True) for topic in manifest.get("topics", [])]
    # Action nodes from actions
    real_actions = [make_action_schema(action) for action in manifest.get("actions", [])]
    # Builtins (treat same as actions)
    builtins = [make_action_schema(builtin) for builtin in manifest.get("builtins", [])]
    all_nodes = composites + topic_actions + real_actions + builtins
    schema = {
        "title": "BehaviorTree",
        "description": (
            "A behavior tree for a robot. The root and all composite nodes must be a Sequence or Selector with a children array. "
            "Action nodes must set type='Action', include a name, and must not have children."
        ),
        "type": "object",
        "properties": {
            "type": {"type": "string", "enum": ["Sequence", "Selector"]},
            "children": {
                "type": "array",
                "items": {"$ref": "#/$defs/BTNode"}
            }
        },
        "required": ["type", "children"],
        "additionalProperties": False,
        "$defs": {
            "BTNode": {
                "anyOf": all_nodes
            }
        }
    }
    return schema

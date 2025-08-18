# schema_builder.py
from ros_prompt.utilities.constants import CapabilityCategory

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
        {
            "type": "object",
            "description": "A Parallel node: runs all child nodes simultaneously; succeeds only if all children succeed.",
            "properties": {
                "type": { "type": "string", "enum": ["Parallel"] },
                "children": {
                    "type": "array",
                    "items": { "$ref": "#/$defs/BTNode" }
                },
            },
            "required": ["type", "children"],
            "additionalProperties": False
        },
    ]

    # Add decorator nodes
    decorators = [
        {
            "type": "object",
            "description": "Timeout decorator: returns FAILURE if its child does not finish within 'duration' seconds.",
            "properties": {
                "type": { "type": "string", "enum": ["Timeout"] },
                "child": { "$ref": "#/$defs/BTNode" },
                "duration": { "type": "number", "minimum": 0.0 }
            },
            "required": ["type", "child", "duration"],
            "additionalProperties": False
        },
        {
            "type": "object",
            "description": "Repeat decorator: repeats its child node a given number of times.",
            "properties": {
                "type": { "type": "string", "enum": ["Repeat"] },
                "child": { "$ref": "#/$defs/BTNode" },
                "num_repeats": { "type": "integer", "minimum": 1 }
            },
            "required": ["type", "child", "num_repeats"],
            "additionalProperties": False
        },
        {
            "type": "object",
            "description": "Retry decorator: re-executes its child if it fails, up to 'num_retries' times.",
            "properties": {
                "type": { "type": "string", "enum": ["Retry"] },
                "child": { "$ref": "#/$defs/BTNode" },
                "num_retries": { "type": "integer", "minimum": 1 }
            },
            "required": ["type", "child", "num_retries"],
            "additionalProperties": False
        },
    ]

    timer_node = {
        "type": "object",
        "description": "A Timer node: waits for a specified duration in seconds before returning SUCCESS.",
        "properties": {
            "type": { "type": "string", "enum": ["Timer"] },
            "duration": { "type": "number", "minimum": 0.0 }
        },
        "required": ["type", "duration"],
        "additionalProperties": False
    }

    action_nodes = [
        make_action_schema(item)
        for category in CapabilityCategory.ordered()
        for item in manifest.get(category, [])
    ]

    all_nodes = composites + decorators + [timer_node] + action_nodes
    schema = {
        "title": "BehaviorTree",
        "description": (
            "A behavior tree for a robot. The root and all composite nodes must be a Sequence or Selector with a children array. "
            "Action nodes must set type='Action', include a name, and must not have children."
        ),
        "type": "object",
        "properties": {
            "type": {"type": "string", "enum": ["Sequence", "Selector", "Parallel"]},
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

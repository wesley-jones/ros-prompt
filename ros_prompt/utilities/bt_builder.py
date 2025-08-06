from platform import node
import py_trees
from ros_prompt.utilities.manifest_helpers import find_manifest_entry, validate_and_coerce_attributes, load_class_from_manifest_entry
from ros_prompt.utilities.ros_type_utils import import_ros_type
from ros_prompt.adapters_py.generic_publisher_adapter import GenericPublisherAdapter
from ros_prompt.behaviors_py.generic_bt_behaviour import GenericPublisherBehaviour
from ros_prompt.adapters_py.generic_action_adapter import GenericActionAdapter
from ros_prompt.behaviors_py.generic_action_behaviour import GenericActionBehaviour
from ros_prompt.adapters_py.generic_service_adapter import GenericServiceAdapter
from ros_prompt.behaviors_py.generic_service_behaviour import GenericServiceBehaviour

def parse_bt_dict(self, bt_dict, manifest_map):

    def build_tree(node):
        node_type = node.get("type")
        name = node.get("name")  # Only present for Action nodes

        # Composite nodes
        if node_type in ["Sequence", "Selector", "Parallel"]:
            # You can add Fallback, Parallel, etc, if you add them to the schema
            if node_type == "Sequence":
                behaviour = py_trees.composites.Sequence(name="Sequence", memory=True)
            elif node_type == "Selector":
                behaviour = py_trees.composites.Selector(name="Selector", memory=True)
            elif node_type == "Parallel":
                behaviour = py_trees.composites.Parallel(name="Parallel")
            else:
                raise ValueError(f"Unknown composite type: {node_type}")
            for child in node.get("children", []):
                child_behaviour = build_tree(child)
                behaviour.add_child(child_behaviour)
            return behaviour

        # --- Decorators ---
        elif node_type == "Timeout":
            child = build_tree(node["child"])
            duration = node["duration"]
            return py_trees.decorators.Timeout(child=child, duration=duration, name="Timeout")
        elif node_type == "Repeat":
            child = build_tree(node["child"])
            num_repeats = node["num_repeats"]
            return py_trees.decorators.Repeat(child=child, num_success=num_repeats, name="Repeat")
        elif node_type == "Retry":
            child = build_tree(node["child"])
            num_retries = node["num_retries"]
            return py_trees.decorators.Retry(child=child, num_failures=num_retries, name="Retry")

        # --- Other node types ---
        if node_type == "Timer":
            return py_trees.timers.Timer(name="Timer", duration=node["duration"])

        # Action nodes (leaf)
        elif node_type == "Action":
            params = node.get("parameters", {})
            # Use name for manifest lookup (topic, action, builtin)
            tag = name  # Always use the LLM-exposed name
            try:
                coerced_params = validate_and_coerce_attributes(tag, params, manifest_map)
            except ValueError as e:
                self.get_logger().error(str(e))
                raise
            cap = find_manifest_entry(tag, manifest_map)
            if not cap:
                raise ValueError(f"Capability '{tag}' not found in manifest.")
            interface = cap.get("interface", "topic_publisher")
            # Reuse your adapter/behavior construction logic
            if interface == "topic_publisher":
                msg_class = import_ros_type(self, cap["type"])
                adapter = GenericPublisherAdapter(self, cap["plugin_name"], msg_class)
                behaviour = GenericPublisherBehaviour(adapter, coerced_params, self, name=tag)
            elif interface == "action_client":
                action_class = import_ros_type(self, cap["type"])
                adapter = GenericActionAdapter(self, cap["plugin_name"], action_class)
                behaviour = GenericActionBehaviour(adapter, coerced_params, self, name=tag)
            elif interface == "service_client":
                srv_class = import_ros_type(self, cap["type"])
                adapter = GenericServiceAdapter(self, cap["plugin_name"], srv_class)
                behaviour = GenericServiceBehaviour(adapter, coerced_params, self, name=tag)
            elif interface == "builtin":
                behaviour_class = load_class_from_manifest_entry(cap)
                behaviour = behaviour_class(node=self, **coerced_params, name=tag)
            else:
                raise ValueError(f"Unknown capability interface: {interface}")
            return behaviour
        else:
            raise ValueError(f"Unknown node type: {node_type}")

    # Root: JSON schema always outputs a Sequence or Selector as the root node
    tree_root = build_tree(bt_dict)
    return py_trees.trees.BehaviourTree(tree_root)

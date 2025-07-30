import py_trees
from ros_prompt.utilities.manifest_helpers import find_manifest_entry, validate_and_coerce_attributes, load_class_from_manifest_entry
from ros_prompt.utilities.ros_type_utils import import_ros_type
from ros_prompt.adapters_py.generic_publisher_adapter import GenericPublisherAdapter
from ros_prompt.behaviors_py.generic_bt_behaviour import GenericPublisherBehaviour
from ros_prompt.adapters_py.generic_action_adapter import GenericActionAdapter
from ros_prompt.behaviors_py.generic_action_behaviour import GenericActionBehaviour

def parse_bt_dict(self, bt_dict, manifest_map):

    def build_tree(node):
        node_type = node.get("type")
        name = node.get("name")  # Only present for Action nodes

        # Composite nodes
        if node_type in ["Sequence", "Selector"]:
            # You can add Fallback, Parallel, etc, if you add them to the schema
            if node_type == "Sequence":
                behaviour = py_trees.composites.Sequence(name="Sequence", memory=False)
            elif node_type == "Selector":
                behaviour = py_trees.composites.Selector(name="Selector")
            else:
                raise ValueError(f"Unknown composite type: {node_type}")
            for child in node.get("children", []):
                child_behaviour = build_tree(child)
                behaviour.add_child(child_behaviour)
            return behaviour

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
            elif interface == "builtin":
                behaviour_class = load_class_from_manifest_entry(cap)
                behaviour = behaviour_class(**coerced_params, name=tag)
            else:
                raise ValueError(f"Unknown capability interface: {interface}")
            return py_trees.decorators.OneShot(
                name=f"OneShot_{tag}",
                child=behaviour,
                policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
            )
        else:
            raise ValueError(f"Unknown node type: {node_type}")

    # Root: JSON schema always outputs a Sequence or Selector as the root node
    tree_root = build_tree(bt_dict)
    return py_trees.trees.BehaviourTree(tree_root)

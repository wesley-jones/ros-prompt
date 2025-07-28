from __future__ import annotations
from typing import Dict, Any
import py_trees
from rclpy.node import Node

# helpers & generic adapters/behaviours – unchanged
from ros_prompt.utilities.manifest_helpers import (
    find_manifest_entry,
    validate_and_coerce_attributes,
    load_class_from_manifest_entry,
    import_ros_type,
)
from ros_prompt.adapters_py.generic_publisher_adapter import GenericPublisherAdapter
from ros_prompt.behaviors_py.generic_bt_behaviour import GenericPublisherBehaviour
from ros_prompt.adapters_py.generic_action_adapter import GenericActionAdapter
from ros_prompt.behaviors_py.generic_action_behaviour import GenericActionBehaviour

# ─────────────────────────────────────────────────────────────────────────────
# 1.  Leaf‑node factory (reuses your manifest logic)
# ─────────────────────────────────────────────────────────────────────────────
def create_behaviour_from_manifest(
    leaf: Any,
    manifest: Dict[str, Any],
    ros_node: Node
) -> py_trees.behaviour.Behaviour:
    tag    = leaf.behaviour             # e.g. "NavigateToPose"
    params = leaf.params or {}
    name   = getattr(leaf, "name", tag)

    # validate & coerce ranges/types
    params = validate_and_coerce_attributes(tag, params, manifest)

    cap = find_manifest_entry(tag, manifest)
    if not cap:
        raise ValueError(f"Capability '{tag}' not found in manifest.")

    interface = cap.get("interface", "topic_publisher")

    if interface == "topic_publisher":
        msg_cls  = import_ros_type(ros_node, cap["type"])
        adapter  = GenericPublisherAdapter(ros_node, cap["name"], msg_cls)
        behaviour = GenericPublisherBehaviour(adapter, params, ros_node, name=name)

    elif interface == "action_client":
        action_cls = import_ros_type(ros_node, cap["type"])
        adapter    = GenericActionAdapter(ros_node, cap["name"], action_cls)
        behaviour  = GenericActionBehaviour(adapter, params, ros_node, name=name)

    elif interface == "builtin":
        behaviour_cls = load_class_from_manifest_entry(cap)
        behaviour     = behaviour_cls(**params, name=name)

    elif interface == "topic_subscriber":
        raise NotImplementedError("Subscriber behaviours not implemented yet")

    else:
        raise ValueError(f"Unknown capability interface: {interface}")

    # wrap each leaf in OneShot (same policy as before)
    return py_trees.decorators.OneShot(
        name=f"OneShot_{behaviour.name}",
        child=behaviour,
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    )

# ─────────────────────────────────────────────────────────────────────────────
# 2.  Recursive builder (works with any schema)
# ─────────────────────────────────────────────────────────────────────────────
def build_py_tree(
    node: Any,                          # Pydantic BTNode instance
    manifest_map: Dict[str, Any],
    ros_node: Node
) -> py_trees.behaviour.Behaviour:

    node_type = getattr(node, "type", None)

    # ── Composites ───────────────────────────────────────────
    if node_type == "Sequence":
        comp = py_trees.composites.Sequence(
            name=getattr(node, "name", "Sequence"),
            memory=getattr(node, "memory", False)
        )
        for child in node.children:
            comp.add_child(build_py_tree(child, manifest_map, ros_node))
        return comp

    if node_type in ("Selector", "Fallback"):
        comp = py_trees.composites.Selector(
            name=getattr(node, "name", node_type),
            memory=getattr(node, "memory", False)
        )
        for child in node.children:
            comp.add_child(build_py_tree(child, manifest_map, ros_node))
        return comp

    if node_type == "Parallel":
        comp = py_trees.composites.Parallel(
            name=getattr(node, "name", "Parallel"),
            policy=py_trees.common.ParallelPolicy(
                success_threshold=getattr(node, "success_threshold", 1),
                failure_threshold=len(node.children)
            )
        )
        for child in node.children:
            comp.add_child(build_py_tree(child, manifest_map, ros_node))
        return comp

    # ── Decorator (single‑child) ────────────────────────────
    if node_type == "Decorator":
        child_bt  = build_py_tree(node.child, manifest_map, ros_node)
        deco_name = node.decorator
        name      = getattr(node, "name", deco_name)
        params    = node.params or {}

        if deco_name == "Inverter":
            return py_trees.decorators.Inverter(child=child_bt, name=name)

        if deco_name == "Repeater":
            n = params.get("count", None)
            return py_trees.decorators.Repeater(child=child_bt, name=name,
                                                n=None if n is None else int(n))

        if deco_name == "Timeout":
            dur = float(params.get("seconds", 5.0))
            return py_trees.decorators.Timeout(child=child_bt, name=name,
                                                duration=dur)

        if deco_name == "FailureIsSuccess":
            return py_trees.decorators.FailureIsSuccess(child=child_bt, name=name)

        if deco_name == "SuccessIsFailure":
            return py_trees.decorators.SuccessIsFailure(child=child_bt, name=name)

        raise ValueError(f"Unsupported decorator: {deco_name}")

    # ── Leaf (manifest‑defined) ─────────────────────────────
    if node_type == "Leaf":
        return create_behaviour_from_manifest(node, manifest_map, ros_node)

    # ── Unknown node type ───────────────────────────────────
    raise TypeError(f"Unhandled BTNode type: {node_type}")

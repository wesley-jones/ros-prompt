# ros_prompt/utilities/json_schema_tools.py
from copy import deepcopy

def inline_refs(schema: dict) -> dict:
    """Inline first‑level $ref’s, leave $defs for deeper recursion, avoid cycles."""
    defs   = schema.get("$defs", {})
    cache  = {}
    seen   = set()

    def _resolve(node):
        if isinstance(node, dict):
            if "$ref" in node:
                ref_key = node["$ref"].split("/")[-1]
                if ref_key in cache:
                    return cache[ref_key]
                if ref_key in seen:           # cycle
                    return node
                seen.add(ref_key)
                expanded = _resolve(deepcopy(defs[ref_key]))
                cache[ref_key] = expanded
                return expanded
            return {k: _resolve(v) for k, v in node.items()}
        if isinstance(node, list):
            return [_resolve(i) for i in node]
        return node

    new_schema = _resolve(schema)
    new_schema["$defs"] = defs     # keep for recursive refs
    return new_schema

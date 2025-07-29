from typing import List, Literal, Union
from pydantic import BaseModel, Field, ConfigDict

# Action parameter models
class CmdVelParameters(BaseModel):
    linear_x: float
    linear_y: float
    linear_z: float
    angular_x: float
    angular_y: float
    angular_z: float
    model_config = ConfigDict(extra="forbid")

class NavigateToPoseParameters(BaseModel):
    x: float
    y: float
    theta: float
    model_config = ConfigDict(extra="forbid")

# Forward declaration for recursion
class BTNode(BaseModel):
    model_config = ConfigDict(extra="forbid")

# Composite nodes
class SequenceNode(BaseModel):
    """A Sequence node: runs child nodes in order, succeeding only if all children succeed."""
    type: Literal["Sequence"]
    children: List["BTNode"]
    model_config = ConfigDict(extra="forbid")
    
class SelectorNode(BaseModel):
    """A Selector node: runs child nodes in order, succeeding if any child succeeds."""
    type: Literal["Selector"]
    children: List["BTNode"]
    model_config = ConfigDict(extra="forbid")

# Action nodes
class CmdVelActionNode(BaseModel):
    """Set robot linear and angular velocities indefinitely until further notice."""
    type: Literal["Action"]
    name: Literal["cmd_vel"]
    parameters: CmdVelParameters
    model_config = ConfigDict(extra="forbid")

class NavigateToPoseActionNode(BaseModel):
    """Navigate the robot to a specified pose in the world."""
    type: Literal["Action"]
    name: Literal["navigate_to_pose"]
    parameters: NavigateToPoseParameters
    model_config = ConfigDict(extra="forbid")

# BTNode is a union for children, but NOT for the root
BTNode = Union[
    SequenceNode,
    SelectorNode,
    CmdVelActionNode,
    NavigateToPoseActionNode,
]

# Patch up references for recursion
SequenceNode.model_rebuild()
SelectorNode.model_rebuild()

# ---- Root must be SequenceNode or SelectorNode only ----

# Example:
class BehaviorTree(BaseModel):
    type: Literal["Sequence", "Selector"]
    children: List[BTNode]

def get_bt_tool_schema():
    """
    Returns the OpenAI function-calling tool schema for the BehaviorTree.
    """
    # Guarantee OpenAI compliance:
    # schema["additionalProperties"] = False
    return {
        "name": "BehaviorTree",
        "description": (
            "A behavior tree for a robot. The root must be a Sequence or Selector node."
        ),
        "parameters": BehaviorTree.model_json_schema()
    }

from typing import List, Dict, Any
from pydantic import BaseModel, Field, field_validator, ConfigDict
from pydantic import RootModel

class BTNode(BaseModel):
    """A node in a behavior tree for a robot."""
    type: str = Field(..., description="The node type: 'Sequence', 'Selector', or 'Action'.")
    name: str = Field(..., description="For actions, the action name (e.g., 'cmd_vel', 'navigate_to_pose'); for composites, use empty string ''.")
    parameters: Dict[str, Any] = Field(..., description="Parameters for the action; empty object {} for composites.")
    children: List["BTNode"] = Field(..., description="Child nodes. For composites, one or more children; for actions, must be an empty list [].")
    
    # Optionally add validation here
    @field_validator('children')
    def children_match_type(cls, v, info):
        node_type = info.data.get('type')
        if node_type == "Action" and v:
            raise ValueError("Action nodes must have an empty children list.")
        if node_type in {"Sequence", "Selector"} and not v:
            raise ValueError("Composites must have one or more children.")
        return v
    
    @field_validator('name')
    def name_match_type(cls, v, info):
        node_type = info.data.get('type')
        if node_type == "Action" and not v:
            raise ValueError("Action nodes must have a non-empty name.")
        if node_type in {"Sequence", "Selector"} and v != "":
            raise ValueError("Composite nodes must have empty string for name.")
        return v

    @field_validator('parameters')
    def params_match_type(cls, v, info):
        node_type = info.data.get('type')
        if node_type == "Action" and not v:
            raise ValueError("Action nodes must have parameters.")
        if node_type in {"Sequence", "Selector"} and v != {}:
            raise ValueError("Composite nodes must have parameters as empty dict.")
        return v
    
    model_config = ConfigDict(arbitrary_types_allowed=True)

class BehaviorTree(RootModel[BTNode]):
    """A behavior tree for a robot."""
    @classmethod
    def openai_schema(cls) -> dict:
        schema = cls.model_json_schema()
        schema["title"] = "BehaviorTree"
        schema["description"] = (
            "A behavior tree for a robot. All nodes must have type, name, parameters, and children. "
            "For Action nodes, set children=[] and provide a meaningful name and parameters. "
            "For Sequence/Selector nodes, set name='' (empty string), parameters={}, and provide one or more children in children."
        )
        return schema
    
BTNode.model_rebuild()

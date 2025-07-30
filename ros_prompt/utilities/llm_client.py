# llm_client.py
import os
from langchain_openai import ChatOpenAI
from ros_prompt.utilities.bt_schema import build_behavior_tree_schema
import jsonschema

class LLMClient:
    def __init__(self, node):
        self.logger = node.get_logger()
        self.api_key = os.getenv("ROS_PROMPT_OPENAI_KEY")
        if not self.api_key:
            raise RuntimeError("Env var ROS_PROMPT_OPENAI_KEY is not set")

    def query_langchain(self, user_prompt: str, manifest: dict, world_context: str = ""):
        schema_dict = build_behavior_tree_schema(manifest)
        self.logger.info(f"Using BehaviorTree schema: {schema_dict}")

        llm = ChatOpenAI(model="gpt-4.1", temperature=0, api_key=self.api_key)

        # 5. Pass the **dict** to with_structured_output
        structured_llm = llm.with_structured_output(
            schema=schema_dict,
            strict=True,
            method="function_calling"
        )
        prompt = f"""
Generate a behavior tree for the provided task in JSON format that matches the provided schema. 

Node types:
- Sequence: Executes its children in order; succeeds only if all children succeed.
- Selector: Tries its children in order; succeeds if any child succeeds.
- Action: Leaf node. Use only supported actions.

Supported actions (Action nodes):
- cmd_vel: Sends velocity commands (linear_x, linear_y, linear_z, angular_x, angular_y, angular_z).
- navigate_to_pose: Navigates the robot to a given pose (x, y, theta).

Every Action node must have the correct name and all required parameters.

Only use the above actions; do not invent new ones.
Task: {user_prompt}
"""
        self.logger.info(f"LLM prompt: {prompt}")
        result_dict = structured_llm.invoke(prompt)
        self.logger.info(f"LLM response: {result_dict}")
        jsonschema.validate(instance=result_dict, schema=schema_dict)
        return result_dict

# llm_client.py
import os
from langchain_openai import ChatOpenAI
from ros_prompt.utilities.bt_schema import build_behavior_tree_schema
from ros_prompt.utilities.manifest_helpers import manifest_summary
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
        # Auto-generate supported actions listing
        actions_list = manifest_summary(manifest)
        prompt = f"""
Generate a behavior tree for the provided task in JSON format that matches the provided schema. 

Node types:
- Sequence: Executes its children in order; returns SUCCESS only if all children return SUCCESS. If any child returns FAILURE, the Sequence returns FAILURE. If a child returns RUNNING, the Sequence returns RUNNING.
- Selector: Executes its children in order; returns SUCCESS if any child returns SUCCESS. If all children return FAILURE, the Selector returns FAILURE. If a child returns RUNNING, the Selector returns RUNNING.
- Action: A leaf node that performs a single robot command. Returns RUNNING while executing, and then returns SUCCESS or FAILURE when done.

Node statuses:
- SUCCESS: The node completed its task successfully.
- FAILURE: The node or one of its children failed to complete the task.
- RUNNING: The node or one of its children is still working.

Supported actions (Action nodes):
{actions_list}

Every Action node must have the correct name and all required parameters.

Only use the above actions; do not invent new ones.

Task: {user_prompt}
"""
        self.logger.info(f"LLM prompt: {prompt}")
        result_dict = structured_llm.invoke(prompt)
        self.logger.info(f"LLM response: {result_dict}")
        jsonschema.validate(instance=result_dict, schema=schema_dict)
        return result_dict

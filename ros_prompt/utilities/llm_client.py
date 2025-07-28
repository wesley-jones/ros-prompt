# llm_client.py
from curses import wrapper
import os
import requests
from langchain_openai import ChatOpenAI
from ros_prompt.utilities.bt_schema import BehaviorTree
from ros_prompt.utilities.json_schema_tools import inline_refs

class LLMClient:
    def __init__(self, node, api_key=None):
        self.node = node
        self.logger = node.get_logger()
        # self.logger.info("Initializing LLMClient...")
        self.api_key = api_key
        self.model_type = "langchain"  # Default model type
        self.api_key = os.getenv("ROS_PROMPT_OPENAI_KEY")
        if not self.api_key:
            raise RuntimeError("Env var ROS_PROMPT_OPENAI_KEY is not set")
        

    def query_langchain(self, user_prompt: str, manifest: dict, world_context: str = ""):

        behavior_tree_schema = {
            "title": "BehaviorTree",
            "description": (
                "A behavior tree for a robot. The root and all composite nodes must be a Sequence or Selector with a children array. "
                "Action nodes must set type='Action', include a name (either 'cmd_vel' or 'navigate_to_pose'), "
                "and must not have children (use an empty array for children in actions for schema compatibility)."
            ),
            "type": "object",
            "properties": {
                "type": {
                    "type": "string",
                    "description": "The node type (Sequence, Selector, or Action)."
                },
                "name": {
                    "type": "string",
                    "description": "The name of the action (for type Action)."
                },
                "parameters": {
                    "type": "object",
                    "description": "Parameters for the action node.",
                    "additionalProperties": False     # <-- REQUIRED!
                },
                "children": {
                    "type": "array",
                    "items": {"$ref": "#/$defs/BTNode"},
                    "description": "Child nodes of this node."
                }
            },
            "required": ["type", "children", "name"],
            "additionalProperties": False,
            "$defs": {
                "BTNode": {
                    "type": "object",
                    "properties": {
                        "type": {"type": "string"},
                        "name": {"type": "string"},
                        "parameters": {
                            "type": "object",
                            "additionalProperties": False   # <-- REQUIRED!
                        },
                        "children": {
                            "type": "array",
                            "items": {"$ref": "#/$defs/BTNode"}
                        }
                    },
                    "required": ["type", "children", "name"],
                    "additionalProperties": False
                }
            }
        }

        pydantic_schema = BehaviorTree.openai_schema()
        self.logger.info(f"Using BehaviorTree schema: {pydantic_schema}")



        llm = ChatOpenAI(model="gpt-4.1", temperature=0, api_key=self.api_key)

        # 5. Pass the **dict** to with_structured_output
        structured_llm = llm.with_structured_output(
            schema=pydantic_schema,         # <-- dict, NOT BaseModel
            strict=True,
            method="function_calling"
        )
        prompt = """
Generate a behavior tree as a JSON object that matches the provided schema. The root node should be a Sequence. Use these node types: Sequence, Selector, and Action. Each node must have a "type" field. "children" is an array of child nodes. Actions are leaf nodes and do not have children.

Task: {user_prompt}
"""
        result = structured_llm.invoke(prompt)
        self.logger.info(f"LLM response: {result}")
        return result

        # 3. Automatic retry on parse failure
        # parser = RetryOutputParser(parser=structured_llm.output_parser, llm=llm, max_retries=1)

        # 4. Prompt template
#         prompt_1 = PromptTemplate.from_template("""
# You are ROS Prompt's planner. Return ONE behaviour tree JSON
# that starts with:
# {{
#   ""type"": "Sequence",
#   "children": [ … ]
# }}
# Valid node types inside children are:
# • Sequence          - ordered steps
# • Selector          - try children until one succeeds
# • Timeout {{seconds}} - limit a single child's runtime
# • Repeater {{count}}  - run a single child N times
# • Each capability leaf from the manifest with its own parameters
# No other keys are allowed.
# User request: {user_prompt}
# """)

        # 5. Load an example tree (optional)
        # example_bt = {
        #     "type": "Sequence",
        #     "children": [
        #         {
        #             "type": "NavigateToPose",
        #             "x": 0.6,
        #             "y": 1.5,
        #             "yaw": 0.1
        #         }
        #     ]
        # }
        # reply = (prompt | structured_llm).invoke({
        #     "user_prompt": user_prompt,
        #     # "schema": json.dumps(inline_schema)     <-- REMOVE
        #     # "example": example_bt                  <-- keep if desired
        # })
        # reply = chain.invoke({
        #     "user_prompt":   user_prompt,
        #     "world_context": world_context,
        #     "schema":        json.dumps(inline_schema, indent=2),   # ← here
        #     "example":       example_bt,
        # })
        # self.logger.info(json.dumps(inline_schema, indent=2))
        # reply = None

        # return reply

    def query_llm(self, system_prompt, user_prompt, final_instructions=None, world_state=None, max_tokens=256, temperature=0.2, timeout=20):
        # self.logger.info(f"Querying LLM with system prompt: {system_prompt}, user prompt: {user_prompt}, final instructions: {final_instructions}, world state: {world_state}")
        if self.model_type == "llama":
            self.logger.info("Using Llama model for query.")
            # Build the prompt for Llama
            prompt = self._build_llama_prompt(system_prompt, user_prompt, world_state, final_instructions)
            self.logger.info(f"Constructed prompt: {prompt}")
            # Query the Llama model
            return self._query_llama(prompt, max_tokens, temperature, timeout)
        elif self.model_type == "deepseek":
            # Build the prompt for DeepSeek
            prompt = self._build_deepseek_prompt(system_prompt, user_prompt, world_state)
            # Query the DeepSeek model
            return self._query_deepseek(prompt)
        elif self.model_type == "openai":
            # Build the prompt for OpenAI
            self.logger.info("Using OpenAI model for query.")
            return self._query_openai(system_prompt, user_prompt, final_instructions, world_state)

    def _query_openai(self, system_prompt, user_prompt, final_instructions="", world_state=""):
        from openai import OpenAI
        import os

        api_key = os.environ.get("ROS_PROMPT_OPENAI_KEY")
        client = OpenAI(api_key=api_key)

        response = client.chat.completions.create(
            model="gpt-4.1-mini",
            store=True,
            messages=self._build_deepseek_prompt(
                system_prompt=system_prompt + final_instructions,
                user_prompt=user_prompt,
                world_state=world_state
            ),
            stream=False
        )
        return response.choices[0].message.content

    def _query_deepseek(self, prompt):
        from openai import OpenAI

        DEEPSEEK_SERVER_URL = "https://api.deepseek.com"
        DEEPSEEK_API_KEY = "your_deepseek_api_key_here"  # Replace with your actual DeepSeek API key

        client = OpenAI(api_key=DEEPSEEK_API_KEY, base_url=DEEPSEEK_SERVER_URL)

        response = client.chat.completions.create(
            model="deepseek-reasoner",
            messages=[
                {"role": "system", "content": "You are a helpful assistant"},
                {"role": "user", "content": "Hello"},
            ],
            stream=False
        )
        return response.choices[0].message.content

    def _query_llama(self, prompt, max_tokens=256, temperature=0.2, timeout=20):
        LLAMA_SERVER_URL = "http://localhost:8000/v1/completions"  # Update if different
        LLM_MODEL_NAME = "mistral"  # Change to your model's id if needed

        headers = {"Content-Type": "application/json"}
        payload = {
            "model": LLM_MODEL_NAME,
            "prompt": prompt,
            "max_tokens": max_tokens,
            "temperature": temperature
        }
        try:
            response = requests.post(LLAMA_SERVER_URL, headers=headers, json=payload, timeout=timeout)
            response.raise_for_status()
            data = response.json()
            completion = data['choices'][0]['text']
            return completion.strip()
        except requests.ConnectionError:
            self.logger.error(f"LLAMA server at {LLAMA_SERVER_URL} is not running or cannot be reached.")
            return None
        except requests.Timeout:
            self.logger.error("Request to LLAMA server timed out.")
            return None
        except requests.HTTPError as http_err:
            self.logger.error(f"HTTP error from LLAMA server: {http_err}")
            return None
        except Exception as e:
            self.logger.error(f"Unexpected error calling LLAMA server: {e}")
            return None

    def _build_llama_prompt(self, system_prompt, user_prompt, world_state=None, final_instructions=None):
        prompt = ""
        if system_prompt:
            prompt += f"{system_prompt}\n"
        prompt += f"{user_prompt}\n"
        if world_state:
            prompt += f"World state: {world_state}\n\n"        
        prompt += final_instructions if final_instructions else ""
        return prompt
    
    def _build_deepseek_prompt(self, system_prompt, user_prompt, world_state=None):
        prompt = []
        prompt.append({"role": "system", "content": f"{system_prompt}\n\n"})
        prompt.append({"role": "user", "content": (
            f"User: {user_prompt}\n"
            f"World state: {world_state}"
        )})        
        return prompt

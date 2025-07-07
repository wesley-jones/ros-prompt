# llm_client.py
import requests

class LLMClient:
    def __init__(self, node, api_key=None):
        self.node = node
        self.logger = node.get_logger()
        # self.logger.info("Initializing LLMClient...")
        self.api_key = api_key
        self.model_type = "llama"  # Default model type

    def query_llm(self, system_prompt, user_prompt, final_instructions=None, world_state=None, max_tokens=256, temperature=0.2, timeout=20):
        self.logger.info(f"Querying LLM with system prompt: {system_prompt}, user prompt: {user_prompt}, final instructions: {final_instructions}, world state: {world_state}")
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
        print(response.choices[0].message.content)
        
    
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

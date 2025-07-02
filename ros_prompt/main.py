#!/usr/bin/env python3

import sys
from llama_cpp import Llama
import subprocess
import rclpy
from rclpy.node import Node
import requests
import json

def get_installed_ros_packages():
    result = subprocess.run(["ros2", "pkg", "list"], capture_output=True, text=True)
    packages = result.stdout.strip().split('\n')
    return packages

def get_launch_files(package_name):
    result = subprocess.run(["ros2", "pkg", "prefix", package_name], capture_output=True, text=True)
    prefix = result.stdout.strip()
    # This is where launch files live by convention:
    launch_dir = f"{prefix}/share/{package_name}/launch"
    import glob, os
    if os.path.isdir(launch_dir):
        files = glob.glob(f"{launch_dir}/*.launch.py")
        return [os.path.basename(f) for f in files]
    return []

def remote_prompt():
    # LLM server config — update with ThinkPad IP if needed
    llm_url = 'http://192.168.1.236:8000/v1/completions'
    headers = {"Content-Type": "application/json"}
    prompt_text = """You are an autonomous vehicle navigating an ordered list of waypoints. Choose the single best action based on the given context:

Waypoints:
1. Go straight for 20 yards
2. Turn left 90 degrees
3. Go straight for 10 yards
4. Turn right 90 degrees
5. Go straight for 20 yards
6. Park at docking station

Current status:
- Current Waypoint: 5
- Distance from next waypoint: 2 yards
- Obstacles: none

Choose from the following actions: [
stop, 
forward, 
backward, 
left 90 degrees, 
right 90 degrees
]

Action: """

    payload = {
            "prompt": prompt_text,
            "max_tokens": 10,
            "temperature": 0.0,
            #"stop": ["\n"]
        }

    try:
        response = requests.post(llm_url, headers=headers, data=json.dumps(payload))
        response.raise_for_status()
        result = response.json()
        llm_output = result["choices"][0]["text"].strip()
        print(f"LLM output: {llm_output}")
        return llm_output

    except Exception as e:
        print(f"Failed to call LLM: {e}")
        return None

def local_prompt():

    # Load the model (do this ONCE, at startup)
    llm = Llama(model_path="/home/wesley-jones/ros2_ws/src/ros_prompt/ros_prompt/models/llama-2-7b.Q4_K_M.gguf")

    # Formulate the prompt
    # system_prompt = "You are a helpful assistant that only outputs ROS 2 CLI commands, nothing else. Given a user request, reply ONLY with the command to run, or reply with 'UNKNOWN' if not possible."
    prompt = """
    {
        High level waypoints: {
            Go straight for 20 yards,
            Turn left 90 degrees,
            go straight for 10 yards,
            turn right 90 degrees,
            got straight for 20 yards,
            park at docking station
        },
        Current waypoint: go straight for 10 yards,
        Distance from next waypoint: 0 yards,
        Obstacles in sight: none,
        possible_actions: {
            stop,
            forward,
            backward,
            left 90 degrees,
            right 90 degrees,
        }
    }
    """
    full_prompt = """You are an autonomous vehicle navigating a set of waypoints. Choose the single best action based on your current position and goal.

Waypoints:
1. Go straight for 20 yards
2. Turn left 90 degrees
3. Go straight for 10 yards (CURRENT)
4. Turn right 90 degrees (NEXT)
5. Go straight for 20 yards
6. Park at docking station

Current status:
- Distance from next waypoint: 0 yards
- Obstacles: none
- Possible actions: [stop, forward, backward, left 90 degrees, right 90 degrees, move to next waypoint]

Question: From the possible actions what is the best action to take?

Answer:"""

    # Get the LLM's output
    output = llm(full_prompt, max_tokens=20, stop=["\n"], stream=False)
    ros_command = output["choices"][0]["text"].strip()
    print(f"LLM output: {ros_command}")
    # print(f"LLM output: {output}")



def main():
    if len(sys.argv) > 1:
        prompt = ' '.join(sys.argv[1:])
        print(f"User prompt: {prompt}")

        # Load the model (do this ONCE, at startup)
        llm = Llama(model_path="/home/wesley-jones/ros2_ws/src/ros_prompt/ros_prompt/models/llama-2-7b.Q4_K_M.gguf")

        # Formulate the prompt
        # system_prompt = "You are a helpful assistant that only outputs ROS 2 CLI commands, nothing else. Given a user request, reply ONLY with the command to run, or reply with 'UNKNOWN' if not possible."
        system_prompt = "You are a helpful assistant that converts user requests into ROS 2 CLI commands."
        full_prompt = f"{system_prompt}\nUser: {prompt}\nROS 2 command:"

        # Get the LLM's output 
        output = llm(full_prompt, max_tokens=64, stop=["\n"], stream=False)
        ros_command = output["choices"][0]["text"].strip()
        print(f"LLM output: {ros_command}")
        print(f"LLM output: {output}")

        # Very basic mapping example
        if "ros2 run" in prompt.lower():
            # (Eventually this command will be dynamically generated)
            print(f"Running: {output}")
            # subprocess.run(["ros2", "launch", "turtlebot4_gazebo", "empty_world.launch.py"])
        else:
            print("Sorry, I don't know how to handle that prompt yet.")
    else:
        print("Usage: ros2 run ros_prompt main -- <your prompt here>")

if __name__ == '__main__':
    remote_prompt()

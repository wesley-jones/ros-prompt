# ros-prompt

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo Sim](https://img.shields.io/badge/Sim-Gazebo%20Ignition-0093DD)](https://gazebosim.org/)
[![Python 3.11+](https://img.shields.io/badge/Python-3.11%2B-3776AB)](https://www.python.org/)
[![Made with py_trees](https://img.shields.io/badge/BT-py_trees-8a2be2)](https://github.com/splintered-reality/py_trees)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Natural language robot control for ROS 2 using LLMs (Mistral, Llama, etc).

## Features
- Understands and plans robot actions from natural language
- Automatic capability discovery (topics, actions, services)
- Intent detection and behavior tree generation
- Modular and extensible ROS 2 package

## Getting Started
### 🔑 Setting your LLM API secret

1. Install ROS Prompt (`pip install ros_prompt` or colcon build).
2. Run `export ROS_PROMPT_OPENAI_KEY="sk-..."` with your own API key.
   • Prefer DeepSeek? `export ROS_PROMPT_DEEPSEEK_KEY="..."`.
3. Launch the stack


## License
MIT


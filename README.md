# ros-prompt

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo Sim](https://img.shields.io/badge/Sim-Gazebo%20Ignition-0093DD)](https://gazebosim.org/)
[![Python 3.11+](https://img.shields.io/badge/Python-3.11%2B-3776AB)](https://www.python.org/)
[![Made with py_trees](https://img.shields.io/badge/BT-py_trees-8a2be2)](https://github.com/splintered-reality/py_trees)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Natural language robot control for ROS 2 using LLMs (LangChain).

#### ROS Prompt Demo: Autonomously switch between SLAM and Navigation mode (click for YouTube)
[![ROS Prompt YouTube Thumbnail](https://img.youtube.com/vi/3zefdX6pDP0/0.jpg)](https://youtu.be/3zefdX6pDP0)

## Features
- Understands and plans robot actions from natural language
- Automatic capability discovery (topics, actions, services)
- Intent detection and behavior tree generation
- Modular and extensible ROS 2 package

## Getting Started
1. Run `export ROS_PROMPT_OPENAI_KEY="sk-..."` with your own API key.
2. pip install py_trees langchain langchain_openai jsonschema
3. Clone ros-prompt into your ROS workspace
4. colcon build --packages-select ros_prompt
5. source install/setup.bash
6. Start your launch files (ie. ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py)
7. ros2 launch ros_prompt ros_prompt.launch.py
8. Publish your messages to /user_input: 
ros2 topic pub /user_input std_msgs/String "data: 'start slam, explore, save map, swith to nav with map, set initial pose, wait for 12 seconds, navigate to position (0.6, 1.5)'" -1


## License
MIT

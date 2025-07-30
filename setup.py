from setuptools import find_packages, setup

package_name = 'ros_prompt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/robot_caps.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/ros_prompt.launch.py'
        ]),
    ],
    install_requires=[
        'setuptools',
        'py_trees',
        'langchain',
        'langchain_openai',
        'jsonschema',
    ],
    zip_safe=True,
    maintainer='wesley-jones',
    maintainer_email='wesoccer2003@yahoo.com',
    description='TODO: Package description',
    license='MIT',
    entry_points={
        'console_scripts': [
            'capability_scanner_node = ros_prompt.nodes.capability_scanner_node:main',
            'planner_node = ros_prompt.nodes.planner_node:main',
            'executor_node = ros_prompt.nodes.executor_node:main',
        ],
    },
)

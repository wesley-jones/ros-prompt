# ros_prompt/adapters_py/builtins/mapping_mode_adapter.py
import py_trees, subprocess, time
from ros_prompt.adapters_py.builtins.lifecycle_helpers import (
        shutdown_nav2, shutdown_slam, nav2_startup_complete,
        launch_cmd_non_blocking, process_alive)

class MappingModeManager(py_trees.behaviour.Behaviour):
    """
    1) shut Nav2 down      (lifecycle SHUTDOWN)
    2) ensure SLAM stopped
    3) launch SLAM Toolbox online_async_launch.py
    4) launch Nav2  slam:=True autostart:=False
    5) call lifecycle STARTUP and wait for ACTIVE
    Success when /map & /slam_toolbox/pose exist.
    """

    def __init__(self, node,
                 slam_launch: str,
                 nav_launch: str,
                 common_args: str,
                 nav_args: str,
                 timeout_sec: int = 45,
                 name="MappingModeManager"):
        super().__init__(name)
        self.get_logger = node.get_logger
        self.slam_launch = slam_launch.split()
        self.nav_launch  = nav_launch.split()
        self.common_args = common_args.split()
        self.nav_args    = nav_args.split()
        self.timeout_sec = timeout_sec
        self._step = 0
        self._nav_proc = self._slam_proc = None
        self._deadline = None

    # ───────── py_trees hooks ───────────────────────────────
    def initialise(self):
        self.get_logger().info("[MappingMode] initialise()")
        shutdown_nav2(self.get_logger)      # fast lifecycle SHUTDOWN
        shutdown_slam(self.get_logger)      # idem
        self._step = 0
        self._deadline = time.time() + self.timeout_sec

    def update(self):
        now = time.time()
        if now > self._deadline:
            self.get_logger().error("[MappingMode] timeout")
            return py_trees.common.Status.FAILURE

        # 0) launch SLAM once Nav2 is down
        if self._step == 0:
            self._slam_proc = launch_cmd_non_blocking(
                ["ros2", "launch"] + self.slam_launch + self.common_args)
            self._step = 1
            return py_trees.common.Status.RUNNING

        # 1) launch Nav2 (slam:=True) once SLAM is publishing /map
        if self._step == 1 and process_alive(self._slam_proc):
            self._nav_proc = launch_cmd_non_blocking(
                ["ros2", "launch"] + self.nav_launch +
                self.common_args + self.nav_args)
            self._step = 2
            return py_trees.common.Status.RUNNING

        # 2) issue lifecycle STARTUP + wait ACTIVE + topics ready
        if self._step == 2:
            if nav2_startup_complete(self.get_logger,
                                     expect_topics=["/map",
                                                    "/slam_toolbox/pose"]):
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING

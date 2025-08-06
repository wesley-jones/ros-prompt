# ros_prompt/adapters_py/builtins/launch_manager.py
from fastapi import params
import py_trees, subprocess, signal, time, os, itertools
from pathlib import Path

class LaunchManager(py_trees.behaviour.Behaviour):
    """
    Generic builtin that:
      • kills existing launch processes matching `match_glob`
      • starts new launch_file with launch_args
      • waits for success_topics
    All params are fed from robot_caps.yaml (static + slots).
    """

    def __init__(self,
                 node,
                 launch_file: str,
                 launch_args: str,
                 match_glob: str,
                 success_topics: str,
                 timeout_sec: int = 30,
                 name="LaunchManager",
                 **kwargs):                 # ← accept any extra slots
        """
        `launch_args` comes in as one space-separated string.
        Any extra kwargs (e.g. map_yaml) are handled below.
        """
        super().__init__(name)
        self.get_logger = node.get_logger
        self.launch_file     = launch_file.split()  # ["bringup_launch.py"]
        self.launch_args     = launch_args.split()      # ["slam:=False", …]
        self.match_glob      = match_glob
        self.success_topics  = success_topics.split(",")
        self.timeout_sec     = timeout_sec
        # --- handle optional slots ---------------------------------
        # map_yaml gets appended as another arg if supplied and non-empty
        map_yaml = kwargs.get("map_yaml")
        if map_yaml:
            self.launch_args.append(f"map:={map_yaml}")

        self.get_logger().info("LaunchManager initialized")

    # ── py_trees lifecycle ─────────────────────────────────────────────
    def initialise(self):
        self._stop_processes()
        cmd = ["ros2", "launch"] + self.launch_file + self.launch_args
        self.get_logger().info("Starting launch command: " + " ".join(cmd))
        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, 
            text=True
        )
        self.get_logger().info("Launch command sent.")
        self._start_time  = time.time()
        count  = 0
        # while count < 200 and time.time() - self._start_time < 30.0:
        #     line = self._proc.stdout.readline()
        #     if line:
        #         self.get_logger().info(f"[Launch Output] {line.rstrip()}")
        #         count += 1
        #     elif self._proc.poll() is not None:
        #         # no data *and* child exited
        #         break
        #     else:
        #         # no data yet, child still running → brief sleep
        #         time.sleep(0.1)

    def update(self):
        if self._all_topics_ready():
            return py_trees.common.Status.SUCCESS
        if time.time() - self._start_time > self.timeout_sec:
            # self._proc.terminate()
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    # ── helpers ────────────────────────────────────────────────────────
    def _stop_processes(self, timeout=25):
        self.get_logger().info(f"Stopping processes matching: {self.match_glob}")
        # 1. SIGINT all matching PIDs
        try:
            pids = subprocess.check_output(["pgrep", "-f", self.match_glob], text=True).split()
            self.get_logger().info(f"Found PIDs: {pids}")
        except subprocess.CalledProcessError as e:
            if e.returncode == 1:
                self.get_logger().info(f"[LaunchMgr] No running processes match '{self.match_glob}'. Continuing.")
                return
            else:
                self.get_logger().error(f"[LaunchMgr] Error finding processes: {e}")
                raise
        
        self.get_logger().info(f"pids: {pids}")
        for pid in pids:
            try:
                os.kill(int(pid), signal.SIGINT)
            except Exception:
                pass

        # 2. Wait until both PIDs and Nav2 nodes disappear
        deadline = time.time() + timeout
        while time.time() < deadline:
            nodes  = subprocess.check_output(["ros2", "node", "list"], text=True).split()
            still_running = any(pid and Path(f"/proc/{pid}").exists() for pid in pids)
            nav2_nodes    = any("/controller_server" in n or "/lifecycle_manager_navigation" in n for n in nodes)
            if not still_running and not nav2_nodes:
                return True
            time.sleep(0.2)

        self.get_logger().info("Processes did not stop in time, force-killing.")

        # 3. Force-kill as last resort
        for pid in pids:
            try:
                os.kill(int(pid), signal.SIGKILL)
            except Exception:
                pass
        return False


    def _all_topics_ready(self):
        topics = subprocess.check_output(["ros2", "topic", "list"], text=True).split()
        self.get_logger().info(f"Current topics: {topics}")
        self.get_logger().info(f"Expected success topics: {self.success_topics}")
        return all(t in topics for t in self.success_topics)

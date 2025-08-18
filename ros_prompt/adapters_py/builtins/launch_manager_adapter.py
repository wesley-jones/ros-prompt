# ros_prompt/adapters_py/builtins/launch_manager.py
import py_trees, subprocess, signal, time, os
from pathlib import Path
from nav2_msgs.srv import ManageLifecycleNodes


_NAV2_LC_CMDS = {"startup": 1, "shutdown": 2, "reset": 3, "pause": 4, "resume": 5}

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
                 # lifecycle (optional)
                #  lifecycle_manager: str | None = None,         # e.g. "/lifecycle_manager_localization"
                #  lifecycle_command: str = "startup",
                #  lifecycle_timeout_sec: int = 20,
                #  wait_for_lifecycle: bool = True,
                 **kwargs):                 # ← accept any extra slots
        """
        `launch_args` comes in as one space-separated string.
        Any extra kwargs (e.g. map_yaml) are handled below.
        """
        super().__init__(name)
        self.node = node
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

        # lifecycle config
        # self.lifecycle_manager = lifecycle_manager
        # self.lifecycle_cmd_str = (lifecycle_command or "startup").lower().strip()
        # self.lifecycle_timeout = lifecycle_timeout_sec
        # self.wait_for_lifecycle = wait_for_lifecycle

        # runtime fields
        # self._lifecycle_client = None
        # self._lifecycle_future = None
        # self._lifecycle_sent   = False
        # self._start_time       = None

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

        # Block until launch process exits, printing logs to the node's logger
        # for line in self._proc.stdout:
        #     self.get_logger().info(line.strip())

        # self._proc.wait()
        self.get_logger().info("Launch command sent.")
        self._start_time  = time.time()

        # Prepare lifecycle client (don’t call yet)
        # if self.lifecycle_manager:
        #     svc_name = f"{self.lifecycle_manager.rstrip('/')}/manage_nodes"
        #     self._lifecycle_client = self.node.create_client(ManageLifecycleNodes, svc_name)
            # don’t block hard; we’ll poll readiness in update()

    def update(self):
        topics_ready = self._all_topics_ready()
        self.get_logger().info(f"Topics ready: {topics_ready}")

        # # Once topics are ready, issue lifecycle (once)
        # if topics_ready and self.lifecycle_manager and not self._lifecycle_sent:
        #     # wait for service (non-blocking poll with short timeout)
        #     if self._lifecycle_client.service_is_ready() or \
        #        self._lifecycle_client.wait_for_service(timeout_sec=0.1):
        #         cmd = _NAV2_LC_CMDS.get(self.lifecycle_cmd_str)
        #         if cmd is None:
        #             self.get_logger().error(f"Unknown lifecycle_command '{self.lifecycle_cmd_str}'")
        #             self._lifecycle_sent = True  # prevent retry spam
        #         else:
        #             req = ManageLifecycleNodes.Request()
        #             req.command = cmd
        #             self.get_logger().info(
        #                 f"Issuing lifecycle '{self.lifecycle_cmd_str}' ({cmd}) to "
        #                 f"{self._lifecycle_client.srv_name}"
        #             )
        #             self._lifecycle_future = self._lifecycle_client.call_async(req)
        #             self._lifecycle_sent = True
        #     else:
        #         # service not ready yet; keep running until it comes up or we time out
        #         pass

        # # If we sent lifecycle, optionally wait for its completion
        # if self._lifecycle_future is not None and self.wait_for_lifecycle:
        #     self.get_logger().info("Waiting for lifecycle response...")
        #     if self._lifecycle_future.done():
        #         if self._lifecycle_future.exception() is None:
        #             resp = self._lifecycle_future.result()
        #             # ManageLifecycleNodes has a 'success' bool
        #             if getattr(resp, "success", True):
        #                 self.get_logger().info(f"Lifecycle call succeeded: {resp}")
        #                 # lifecycle done; now fall through to success logic below
        #                 self._lifecycle_future = None
        #             else:
        #                 self.get_logger().error("Lifecycle call failed: " + str(resp))
        #                 return py_trees.common.Status.FAILURE
        #         else:
        #             self.get_logger().error(f"Lifecycle call failed: {self._lifecycle_future.exception()}")
        #             return py_trees.common.Status.FAILURE
        #     else:
        #         # still waiting for lifecycle response
        #         if time.time() - self._start_time > self.timeout_sec:
        #             self.get_logger().error(f"Lifecycle call timed out after {self.timeout_sec}s")
        #             return py_trees.common.Status.FAILURE
        #         return py_trees.common.Status.RUNNING

        # Success condition
        if topics_ready:
            return py_trees.common.Status.SUCCESS

        # Timeout
        if time.time() - self._start_time > self.timeout_sec:
            self.get_logger().error(f"LaunchManager timed out after {self.timeout_sec}s")
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

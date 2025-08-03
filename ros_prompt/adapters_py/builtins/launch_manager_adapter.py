# ros_prompt/adapters_py/builtins/launch_manager.py
from fastapi import params
import py_trees, subprocess, signal, time

class LaunchManager(py_trees.behaviour.Behaviour):
    """
    Generic builtin that:
      • kills existing launch processes matching `match_glob`
      • starts new launch_file with launch_args
      • waits for success_topics
    All params are fed from robot_caps.yaml (static + slots).
    """

    def __init__(self,
                 launch_file: str,
                 launch_args: str,
                 match_glob: str,
                 success_topics: str,
                 timeout_sec: int = 15,
                 name="LaunchManager",
                 **kwargs):                 # ← accept any extra slots
        """
        `launch_args` comes in as one space-separated string.
        Any extra kwargs (e.g. map_yaml) are handled below.
        """
        super().__init__(name)
        self.launch_file     = launch_file
        self.launch_args     = launch_args.split()      # ["slam:=False", …]
        self.match_glob      = match_glob
        self.success_topics  = success_topics.split(",")
        self.timeout_sec     = timeout_sec
        # --- handle optional slots ---------------------------------
        # map_yaml gets appended as another arg if supplied and non-empty
        map_yaml = kwargs.get("map_yaml")
        if map_yaml:
            self.launch_args.append(f"map:={map_yaml}")

    # ── py_trees lifecycle ─────────────────────────────────────────────
    def initialise(self):
        self._stop_procs()
        cmd = ["ros2", "launch", self.launch_file] + self.launch_args

        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL, 
            stderr=subprocess.STDOUT, 
            text=True
        )
        self._start_time = time.time()

    def update(self):
        if self._all_topics_ready():
            return py_trees.common.Status.SUCCESS
        if time.time() - self._start_time > self.timeout_sec:
            self._proc.terminate()
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    # ── helpers ────────────────────────────────────────────────────────
    def _stop_procs(self):
        subprocess.run(["pkill", "-f", self.match_glob],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _all_topics_ready(self):
        topics = subprocess.check_output(["ros2", "topic", "list"], text=True).split()
        return all(t in topics for t in self.success_topics)

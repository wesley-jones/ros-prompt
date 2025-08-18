# ros_prompt/adapters_py/builtins/run_process_adapter.py
import py_trees, subprocess, shlex
import time, itertools

class RunProcessAdapter(py_trees.behaviour.Behaviour):
    """
    Fire-and-forget subprocess (runs until killed by SIGINT when tree stops).
    """

    def __init__(self, node, cmd: str, name="RunProcessAdapter", **_):
        super().__init__(name)
        self.cmd = shlex.split(cmd)
        self.proc = None

    # py_trees hooks -------------------------------------------------------
    def initialise(self):
        self.proc = subprocess.Popen(self.cmd)

    def update(self):
        if self.proc.poll() is None:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if self.proc and self.proc.poll() is None:
            self.proc.send_signal(subprocess.signal.SIGINT)

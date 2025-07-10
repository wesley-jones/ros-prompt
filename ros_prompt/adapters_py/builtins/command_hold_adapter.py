import py_trees
import time

class CommandHold(py_trees.behaviour.Behaviour):
    def __init__(self, duration=5.0, name="CommandHold"):
        super().__init__(name)
        self.duration = duration
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        if time.time() - self.start_time < self.duration:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

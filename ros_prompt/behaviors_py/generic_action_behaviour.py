import py_trees
import time

class GenericActionBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, adapter, params, node, name="GenericActionBehaviour", timeout=30.0):
        super().__init__(name)
        self.adapter = adapter
        self.params = params
        self.get_logger = node.get_logger
        self.started = False
        self.timeout = timeout
        self.start_time = None

    def initialise(self):
        self.get_logger().info(f"Initialising {self.name} (sending goal): {self.params}")
        self.adapter.execute(timeout=self.timeout,**self.params)
        self.started = True
        self.start_time = time.time()

    def update(self):
        status = self.adapter.check_status()
        self.get_logger().info(f"{self.name} status: {status}")
        if status == "succeeded":
            return py_trees.common.Status.SUCCESS
        elif status in ["rejected", "cancelled"]:
            return py_trees.common.Status.FAILURE
        elif status in ["active", "pending"]:
            if self.start_time and (time.time() - self.start_time) > self.timeout:
                self.get_logger().error(f"{self.name} timed out after {self.timeout}s.")
                self.adapter.cancel_goal()
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING

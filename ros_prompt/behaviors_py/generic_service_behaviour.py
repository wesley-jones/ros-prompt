import time
import py_trees
from py_trees.common import Status


class GenericServiceBehaviour(py_trees.behaviour.Behaviour):
    """
    One-shot asynchronous service call.
    Returns RUNNING until the Future resolves, then SUCCESS/FAILURE.
    """

    def __init__(self, adapter, params, node,
                 name="GenericServiceBehaviour", timeout_sec: float | None = 10.0):
        super().__init__(name)
        self.adapter = adapter
        self.params = params
        self.get_logger = node.get_logger
        self.timeout_sec = timeout_sec
        self.future = None
        self.start = None

    # ------------
    # Life-cycle
    # ------------
    def initialise(self):
        self.get_logger().info(f"Initialising {self.name} with params: {self.params}")
        self.future = self.adapter.execute(**self.params)
        self.start = time.time()

    def update(self) -> Status:
        # Still waiting?
        if not self.future.done():
            if self.timeout_sec and (time.time() - self.start) > self.timeout_sec:
                self.get_logger().error(f"{self.name} timed out")
                return Status.FAILURE
            return Status.RUNNING

        # Completed – inspect result
        if self.future.exception() is None:
            self.get_logger().info(f"{self.name} succeeded: {self.future.result()}")
            return Status.SUCCESS
        else:
            self.get_logger().error(f"{self.name} failed: {self.future.exception()}")
            return Status.FAILURE

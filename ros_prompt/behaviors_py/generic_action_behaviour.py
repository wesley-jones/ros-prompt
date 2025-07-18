import py_trees

class GenericActionBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, adapter, params, node, name="GenericActionBehaviour"):
        super().__init__(name)
        self.adapter = adapter
        self.params = params
        self.get_logger = node.get_logger
        self.started = False

    def initialise(self):
        self.get_logger().info(f"Initialising {self.name} (sending goal): {self.params}")
        self.adapter.execute(**self.params)
        self.started = True

    def update(self):
        status = self.adapter.check_status()
        self.get_logger().info(f"{self.name} status: {status}")
        if status == "succeeded":
            return py_trees.common.Status.SUCCESS
        elif status == "rejected":
            return py_trees.common.Status.FAILURE
        elif status in ["active", "pending"]:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING

import py_trees

class GenericPublisherBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, adapter, params, node, name="GenericPublisherBehaviour"):
        super().__init__(name)
        self.adapter = adapter
        self.params = params
        self.get_logger = node.get_logger
        self.get_logger().info(f"Init {self.name} with params: {self.params}")

    def initialise(self):
        self.get_logger().info(f"Initialising {self.name} with params: {self.params}")
        self.adapter.execute(**self.params)


    def update(self):
        self.get_logger().info(f"Checking status of {self.name}")
        return py_trees.common.Status.SUCCESS

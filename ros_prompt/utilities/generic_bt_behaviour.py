import py_trees

class GenericPublisherBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, adapter, params, name="GenericPublisherBehaviour"):
        super().__init__(name)
        self.adapter = adapter
        self.params = params

    def initialise(self):
        self.adapter.execute(**self.params)

    def update(self):
        return py_trees.common.Status.SUCCESS

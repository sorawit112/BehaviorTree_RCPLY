from py_trees.behaviours import Failure

class AlwaysFailure(Failure):
    def __init__(self, name: str):
        super().__init__(name)
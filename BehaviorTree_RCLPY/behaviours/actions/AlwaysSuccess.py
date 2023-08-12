from py_trees.behaviours import Success

class AlwaysSuccess(Success):
    def __init__(self, name: str):
        super().__init__(name)
from py_trees.behaviours import Running

class AlwaysRunning(Running):
    def __init__(self, name: str):
        super().__init__(name)
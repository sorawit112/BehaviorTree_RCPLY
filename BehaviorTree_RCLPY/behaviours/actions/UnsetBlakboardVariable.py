from py_trees.behaviours import UnsetBlackboardVariable as pyUnsetBlackboardVariable

class UnsetBlackboardVariable(pyUnsetBlackboardVariable):
    def __init__(self, name: str, key: str):
        if not isinstance(key, str):
            raise TypeError(f"key type:{type(key)} not supported")
        super().__init__(name, key)
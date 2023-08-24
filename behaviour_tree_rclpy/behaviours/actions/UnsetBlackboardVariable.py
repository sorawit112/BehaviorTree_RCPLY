from py_trees.behaviours import UnsetBlackboardVariable as pyUnsetBlackboardVariable

class UnsetBlackboardVariable(pyUnsetBlackboardVariable):
    def __init__(self, attr:dict={}):
        name: str = attr['name'] if 'name' in attr.keys() else 'UnsetBlackboardVariable'
        key: str = attr['key'] 
        if not isinstance(key, str):
            raise TypeError(f"key type:{type(key)} not supported")
        super().__init__(name, key)
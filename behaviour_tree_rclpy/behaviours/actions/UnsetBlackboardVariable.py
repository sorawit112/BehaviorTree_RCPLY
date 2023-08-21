from py_trees.behaviours import UnsetBlackboardVariable as pyUnsetBlackboardVariable

class UnsetBlackboardVariable(pyUnsetBlackboardVariable):
    def __init__(self, **kwargs):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'UnsetBlackboardVariable'
        key: str = kwargs['key'] 
        if not isinstance(key, str):
            raise TypeError(f"key type:{type(key)} not supported")
        super().__init__(name, key)
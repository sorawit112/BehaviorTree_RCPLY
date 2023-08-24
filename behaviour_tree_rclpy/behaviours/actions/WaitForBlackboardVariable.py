from py_trees.behaviours import WaitForBlackboardVariable as pyWaitForBlackboardVariable

class WaitForBlackboardVariable(pyWaitForBlackboardVariable):
    def __init__(self, attr:dict={}):
        name: str = attr['name'] if 'name' in attr.keys() else 'WaitForBlackboardVariable'
        variable_name: str = attr['variable_name']
        if not isinstance(variable_name, str):
            raise TypeError(f"variable_name type:{type(variable_name)} not supported")
        super().__init__(name, variable_name)